// Author: Zhuo Ma
// Experiment scheduler: truth -> ideal snapshot -> dropout -> native controller -> plant.
#include "accsim/control.hpp"
#include "accsim/engine.hpp"
#include "accsim/metrics.hpp"
#include "accsim/perception.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace {
using namespace accsim;
namespace fs=std::filesystem;
struct Options {
    fs::path source,output,resources;
    double dt=.01,sensor_period=.05,duration=10,dropout_p=0,time_gap=1.5,set_speed=-1;
    double mean_missing_s=.5;
    std::string dropout_model="markov",markov_init="stationary",controller="aeb";
    double aeb_ttc_s=1.5,aeb_deceleration=.85*9.81;
    bool stop_on_collision=true;
    bool detailed_logs=false; // truth/perceptions/control/metrics/dropout_states CSVs
    std::uint64_t seed=42;
};
std::string trim(const std::string& s) {
    auto first=s.find_first_not_of(" \t\r\n"),last=s.find_last_not_of(" \t\r\n");
    return first==std::string::npos ? "":s.substr(first,last-first+1);
}
double number(const std::string& text) {
    std::size_t used=0; const double value=std::stod(text,&used);
    if(used!=text.size() || !std::isfinite(value)) throw std::invalid_argument("Expected finite numeric value: "+text);
    return value;
}
Options parse(int argc,char** argv) {
    std::map<std::string,std::string> values;
    fs::path config;
    for(int i=1;i<argc;++i) if(std::string(argv[i])=="--config") {
        if(i+1>=argc) throw std::invalid_argument("--config needs a path");
        config=fs::absolute(argv[++i]);
    }
    if(!config.empty()) {
        std::ifstream file(config); if(!file) throw std::runtime_error("Cannot read config: "+config.string());
        std::string line;
        while(std::getline(file,line)) {
            line=trim(line); if(line.empty() || line[0]=='#' || line[0]==';') continue;
            const auto eq=line.find('='); if(eq==std::string::npos) throw std::runtime_error("Config requires key=value");
            auto key=trim(line.substr(0,eq)),value=trim(line.substr(eq+1));
            if(key=="scenario" || key=="output-dir" || key=="esmini-home") {
                fs::path path(value); if(path.is_relative()) value=(config.parent_path()/path).lexically_normal().string();
            }
            values[key]=value;
        }
    }
    for(int i=1;i<argc;++i) {
        const std::string key=argv[i];
        if(key=="--help") {
            std::cout<<"acc_sim --scenario PATH --output-dir NEW_DIRECTORY [--config FILE]\n"
                     <<"  --dropout-p 0..1 --seed 42 --dt .01 --sensor-period .05\n"
                     <<"  --duration 10 --time-gap 1.5 --set-speed MPS --esmini-home RESOURCES\n"
                     <<"  --dropout-model markov|iid --mean-missing .5 --markov-init stationary|normal\n"
                     <<"  --controller aeb|acc --aeb-ttc 1.5 --aeb-deceleration 8.3385\n"
                     <<"  --stop-on-collision true|false (false is for full-horizon truth-reference validation)\n"
                     <<"  --detailed-logs true|false (default false; validation and replay need true)\n"
                     <<"Headless native controller simulation; sim.dat can be viewed with esmini replayer.\n";
            std::exit(0);
        }
        if(key.rfind("--",0)!=0 || i+1>=argc) throw std::invalid_argument("Expected --option value: "+key);
        const auto value=std::string(argv[++i]); if(key!="--config") values[key.substr(2)]=value;
    }
    const std::vector<std::string> known={"scenario","output-dir","esmini-home","dt","sensor-period","duration","dropout-p","time-gap","set-speed","seed","dropout-model","mean-missing","markov-init","controller","aeb-ttc","aeb-deceleration","stop-on-collision","detailed-logs"};
    for(const auto& value:values) if(std::find(known.begin(),known.end(),value.first)==known.end())
        throw std::invalid_argument("Unknown option: "+value.first);
    Options o;
    if(!values.count("scenario") || !values.count("output-dir")) throw std::invalid_argument("--scenario and --output-dir are required");
    o.source=fs::absolute(values.at("scenario")).lexically_normal();
    o.output=fs::absolute(values.at("output-dir")).lexically_normal();
    if(values.count("esmini-home")) o.resources=fs::absolute(values.at("esmini-home"));
    else if(const char* env=std::getenv("ESMINI_RESOURCE_HOME")) o.resources=fs::absolute(env);
    auto assign=[&](const char* key,double& target) { if(values.count(key)) target=number(values.at(key)); };
    assign("dt",o.dt); assign("sensor-period",o.sensor_period); assign("duration",o.duration);
    assign("dropout-p",o.dropout_p); assign("time-gap",o.time_gap); assign("set-speed",o.set_speed);
    assign("mean-missing",o.mean_missing_s);
    assign("aeb-ttc",o.aeb_ttc_s); assign("aeb-deceleration",o.aeb_deceleration);
    if(values.count("controller")) o.controller=values.at("controller");
    if(o.controller!="acc" && o.controller!="aeb") throw std::invalid_argument("controller must be aeb or acc");
    if(values.count("stop-on-collision")) {
        const auto& value=values.at("stop-on-collision");
        if(value!="true" && value!="false") throw std::invalid_argument("stop-on-collision must be true or false");
        o.stop_on_collision=value=="true";
    }
    if(values.count("detailed-logs")) {
        const auto& value=values.at("detailed-logs");
        if(value!="true" && value!="false") throw std::invalid_argument("detailed-logs must be true or false");
        o.detailed_logs=value=="true";
    }
    if(o.aeb_ttc_s<=0 || o.aeb_deceleration<=0) throw std::invalid_argument("Invalid AEB parameters");
    if(o.controller=="aeb" && values.count("set-speed"))
        throw std::invalid_argument("AEB nominal driver holds current speed; --set-speed belongs to ACC");
    if(values.count("dropout-model")) o.dropout_model=values.at("dropout-model");
    if(values.count("markov-init")) o.markov_init=values.at("markov-init");
    if(o.dropout_model!="markov"&&o.dropout_model!="iid") throw std::invalid_argument("dropout-model must be markov or iid");
    if(o.markov_init!="stationary"&&o.markov_init!="normal") throw std::invalid_argument("markov-init must be stationary or normal");
    if(o.dropout_model=="markov") markov_parameters(o.dropout_p,o.mean_missing_s,o.sensor_period);
    if(values.count("seed")) {
        const auto& text=values.at("seed"); std::size_t used=0;
        if(text.empty() || text[0]=='-') throw std::invalid_argument("seed must be an unsigned integer");
        o.seed=std::stoull(text,&used); if(used!=text.size()) throw std::invalid_argument("Invalid seed");
    }
    if(o.dt<=0 || o.dt>.1 || o.sensor_period<o.dt || o.duration<=0 || o.time_gap<=0 || o.dropout_p<0 || o.dropout_p>1 || o.set_speed<-1)
        throw std::invalid_argument("Invalid simulation parameters");
    if(std::abs(o.sensor_period/o.dt-std::round(o.sensor_period/o.dt))>1e-8 ||
       std::abs(o.duration/o.dt-std::round(o.duration/o.dt))>1e-8)
        throw std::invalid_argument("sensor-period and duration must be integer multiples of dt");
    if(!fs::is_regular_file(o.source)) throw std::runtime_error("Original scenario not found");
    if(fs::exists(o.output) && !fs::is_empty(o.output)) throw std::runtime_error("Output directory is not empty; use a NEW run directory");
    return o;
}
std::string json(const std::string& text) {
    std::ostringstream s; s<<'"';
    for(unsigned char c:text) {
        switch(c) {
        case '"':s<<"\\\"";break; case '\\':s<<"\\\\";break;
        case '\n':s<<"\\n";break;case '\r':s<<"\\r";break;case '\t':s<<"\\t";break;
        default: if(c<32) s<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<int>(c)<<std::dec;
                 else s<<static_cast<char>(c);
        }
    }
    return s.str()+'"';
}
std::string numeric(double value) {
    if(!std::isfinite(value)) return "null";
    std::ostringstream s; s<<std::setprecision(17)<<value; return s.str();
}
std::string event_numeric(double value) { return value<0 ? "null":numeric(value); }
std::ofstream open(const fs::path& path) {
    std::ofstream file(path,std::ios::binary); if(!file) throw std::runtime_error("Cannot write "+path.string());
    file.exceptions(std::ios::badbit|std::ios::failbit); file<<std::setprecision(17); return file;
}
std::string fingerprint(const fs::path& path) {
    std::ifstream in(path,std::ios::binary); if(!in) throw std::runtime_error("Cannot fingerprint input");
    std::uint64_t hash=UINT64_C(14695981039346656037); char c;
    while(in.get(c)) { hash^=static_cast<unsigned char>(c); hash*=UINT64_C(1099511628211); }
    std::ostringstream s; s<<std::hex<<std::setw(16)<<std::setfill('0')<<hash; return s.str();
}
void write_config(const Options& o,const PreparedScenario& scene,const AccConfig& acc) {
    auto file=open(o.output/"run_config.json");
    const auto transitions=o.dropout_model=="markov"?markov_parameters(o.dropout_p,o.mean_missing_s,o.sensor_period):MarkovParameters{};
    file<<"{\n  \"schema_version\": 3,\n  \"author\": \"Zhuo Ma\",\n"
        <<"  \"controller\": "<<json(o.controller=="acc"?"esmini::ControllerACC::Step":
            "esmini::ControllerALKS_R157SM::ReferenceDriver AEB component")<<",\n"
        <<"  \"stop_on_collision\": "<<(o.stop_on_collision?"true":"false")<<",\n"
        <<"  \"detailed_logs\": "<<(o.detailed_logs?"true":"false")<<",\n"
        <<"  \"controller_kind\": "<<json(o.controller)<<",\n"
        <<"  \"aeb_ttc_s\": "<<(o.controller=="aeb"?numeric(o.aeb_ttc_s):"null")<<",\n"
        <<"  \"aeb_max_deceleration_mps2\": "<<(o.controller=="aeb"?numeric(o.aeb_deceleration):"null")<<",\n"
        <<"  \"aeb_ramp_s\": "<<(o.controller=="aeb"?"0.6":"null")<<",\n"
        <<"  \"aeb_scope\": "<<json(o.controller=="aeb"?
            "Upstream Process + UpdateAEB + ReactCritical; human-driver and cruise branches excluded; latch until stop":
            "not applicable")<<",\n"
        <<"  \"esmini_tag\": \"v3.8.1\",\n  \"esmini_commit\": \"19d26b68f7f473de4e55046a794b5fd2f91c58c8\",\n"
        <<"  \"source_scenario\": "<<json(scene.source.string())<<",\n"
        <<"  \"source_fnv1a64\": "<<json(fingerprint(scene.source))<<",\n"
        <<"  \"road_fnv1a64\": "<<json(fingerprint(scene.road))<<",\n"
        <<"  \"generated_scenario\": "<<json(scene.generated.string())<<",\n"
        <<"  \"dt_s\": "<<o.dt<<",\n  \"sensor_period_s\": "<<o.sensor_period<<",\n"
        <<"  \"duration_s\": "<<o.duration<<",\n  \"dropout_probability\": "<<o.dropout_p<<",\n"
        <<"  \"dropout_model\": "<<json(o.dropout_model)<<",\n"
        <<"  \"mean_missing_s\": "<<(o.dropout_model=="markov"?numeric(o.mean_missing_s):"null")<<",\n"
        <<"  \"markov_initialization\": "<<json(o.markov_init)<<",\n"
        <<"  \"markov_alpha\": "<<(o.dropout_model=="markov"?numeric(transitions.enter_missing):"null")<<",\n"
        <<"  \"markov_beta\": "<<(o.dropout_model=="markov"?numeric(transitions.recover):"null")<<",\n"
        <<"  \"seed\": "<<o.seed<<",\n  \"time_gap_s\": "<<acc.time_gap_s<<",\n"
        <<"  \"set_speed_mps\": "<<acc.set_speed_mps<<",\n  \"initial_ego_speed_mps\": "<<scene.reference.initial_speed()<<",\n"
        <<"  \"max_acceleration_mps2\": "<<acc.max_acceleration<<",\n  \"max_deceleration_mps2\": "<<acc.max_deceleration<<",\n"
        <<"  \"sensor_fov_rad\": "<<pi/3<<",\n  \"sensor_far_m\": 60,\n"
        <<"  \"ego_state_source\": \"exact simulation truth\",\n"
        <<"  \"plant\": \"bounded constant-acceleration longitudinal kinematics\",\n"
        <<"  \"lateral_driver\": \"ideal spatial-path following\",\n"
        <<"  \"dropout_scope\": "<<json(o.dropout_model=="markov"?
            "independent target chains; one transition per perception refresh, even outside sensor FOV":
            "one independent draw per detected object per perception frame")<<",\n"
        <<"  \"missing_policy\": "<<json(o.controller=="acc"?
            "empty new frame; no tracker; ACC free-cruise behavior":
            "empty new frame; no tracker; no new AEB trigger while missing; active AEB continues until stop")<<",\n"
        <<"  \"evaluation\": "<<json(o.stop_on_collision?
            "truth-only upright oriented bounding boxes; stop at first contact":
            "truth-only boxes; continue without collision response ONLY for target-truth reference validation")<<"\n}\n";
}
void write_truth(std::ofstream& file,double time,const ObjectState& s,const char* role) {
    file<<time<<','<<s.id<<','<<role<<','<<s.pose.x<<','<<s.pose.y<<','<<s.pose.z<<','<<s.pose.h<<','<<s.speed<<','
        <<s.box.length<<','<<s.box.width<<','<<s.box.height<<','<<s.box.cx<<','<<s.box.cy<<','<<s.box.cz<<','
        <<s.object_type<<','<<s.object_category<<','
        <<s.velocity.x<<','<<s.velocity.y<<','<<s.velocity.z<<','
        <<s.acceleration.x<<','<<s.acceleration.y<<','<<s.acceleration.z<<'\n';
}
void write_perception(std::ofstream& file,const PerceptionFrame& raw,const DropoutResult& processed) {
    if(raw.objects.empty()) {
        file<<raw.sequence<<','<<raw.measurement_time_s<<','<<raw.delivery_time_s<<",,0,0,0";
        for(int i=0;i<12;++i) file<<',';
        file<<'\n';
    }
    for(const auto& object:raw.objects) {
        const bool dropped=std::find(processed.dropped_ids.begin(),processed.dropped_ids.end(),object.track_id)!=processed.dropped_ids.end();
        file<<raw.sequence<<','<<raw.measurement_time_s<<','<<raw.delivery_time_s<<','<<object.track_id<<",1,"<<(!dropped)<<','<<dropped<<','
            <<object.pose.x<<','<<object.pose.y<<','<<object.speed<<',';
        if(!dropped) file<<object.pose.x<<','<<object.pose.y<<','<<object.speed;
        else file<<",,";
        file<<','<<object.object_type<<','<<object.object_category<<','<<object.box.length<<','<<object.box.width<<',';
        if(!dropped) file<<object.object_type<<','<<object.object_category;
        else file<<',';
        file<<'\n';
    }
}
// Per-run CSV files, opened together with their header rows. results.csv is always
// written; the detailed logs only when requested (validation and replay need them).
struct RunLogs {
    bool detailed=false;
    std::ofstream truth,perception,control,metrics,dropout_states,results;
};
RunLogs open_run_logs(const fs::path& output,bool detailed) {
    RunLogs logs;
    logs.detailed=detailed;
    if(detailed) {
        logs.truth=open(output/"truth.csv"); logs.perception=open(output/"perceptions.csv");
        logs.control=open(output/"control.csv"); logs.metrics=open(output/"metrics.csv");
        logs.dropout_states=open(output/"dropout_states.csv");
        logs.dropout_states<<"sequence,measurement_time_s,track_id,missing,changed,run_length_frames,raw_detected\n";
        logs.truth<<"time_s,object_id,role,x_m,y_m,z_m,h_rad,speed_mps,length_m,width_m,height_m,center_x_m,center_y_m,center_z_m,object_type,object_category,vx_mps,vy_mps,vz_mps,ax_mps2,ay_mps2,az_mps2\n";
        logs.perception<<"sequence,measurement_time_s,delivery_time_s,track_id,raw_detected,observed,dropped,raw_x_m,raw_y_m,raw_speed_mps,observed_x_m,observed_y_m,observed_speed_mps,raw_object_type,raw_object_category,raw_length_m,raw_width_m,observed_object_type,observed_object_category\n";
        logs.control<<"time_s,perception_sequence,measurement_time_s,age_s,fresh,lead_id,observed_gap_m,requested_acceleration_mps2,desired_speed_mps,applied_acceleration_mps2,limited,close_gap_stop,aeb_active,observed_ttc_s\n";
        logs.metrics<<"time_s,collision,engine_collision,minimum_distance_m,minimum_gap_m,ttc_s\n";
    }
    logs.results=open(output/"results.csv");
    logs.results<<"step,time_s,ego_x_m,ego_y_m,ego_heading_rad,ego_speed_mps,ego_progress_m,"
                   "sensor_refreshed,perception_sequence,perception_age_s,ideal_detections,observed_detections,dropped_detections,"
                   "selected_target_id,selected_target_gap_m,selected_target_ttc_s,"
                   "controller_updated,desired_speed_mps,requested_acceleration_mps2,applied_acceleration_mps2,acceleration_limited,"
                   "aeb_active,close_gap_stop,"
                   "truth_min_gap_m,truth_min_gap_target_id,truth_min_ttc_s,truth_min_distance_m,collision\n";
    return logs;
}
void write_metrics(std::ofstream& file,double time,const StepMetrics& metrics,bool engine_collision) {
    file<<time<<','<<metrics.collision<<','<<engine_collision<<','<<metrics.minimum_distance_m<<','<<metrics.gap_m<<','<<metrics.ttc_s<<'\n';
}
// One row per registered Markov chain, written after the chains advanced for this frame.
void write_dropout_states(std::ofstream& file,const PerceptionFrame& ideal_frame,const MarkovDropout& markov_dropout) {
    for(const auto& entry:markov_dropout.states()) {
        const auto& state=entry.second;
        const bool detected=std::any_of(ideal_frame.objects.begin(),ideal_frame.objects.end(),
            [&](const Observation& object){return object.track_id==entry.first;});
        file<<ideal_frame.sequence<<','<<ideal_frame.measurement_time_s<<','<<entry.first<<','<<state.missing<<','
            <<state.changed<<','<<state.run_length<<','<<detected<<'\n';
    }
}
void write_control(std::ofstream& file,double time,const PerceptionFrame& cached_observations,bool is_sensor_frame,
                   const ControlRequest& request,const AppliedCommand& applied) {
    file<<time<<','<<cached_observations.sequence<<','<<cached_observations.measurement_time_s<<','
        <<time-cached_observations.measurement_time_s<<','<<is_sensor_frame<<','
        <<request.lead_id<<','<<request.observed_gap_m<<','<<request.acceleration_mps2<<','<<request.desired_speed_mps<<','
        <<applied.acceleration_mps2<<','<<applied.limited<<','<<request.close_gap_stop<<','<<request.aeb_active<<','<<request.observed_ttc_s<<'\n';
}
// Counts of the cached (filtered) frame; unavailable until the first sensor refresh.
struct CachedFrameCounts {
    bool available=false;
    std::size_t ideal_detections=0,dropped_detections=0;
};
// Writes the value, or an empty cell when it is unavailable (non-finite).
void write_finite(std::ofstream& file,double value) { if(std::isfinite(value)) file<<value; }
// One results.csv row per loop iteration. A terminal row has no new command:
// request/applied are null and their columns stay empty.
void write_result(std::ofstream& file,std::uint64_t step,const WorldTruthFrame& world,double ego_progress_m,
                  bool sensor_refreshed,const PerceptionFrame& cached_observations,const CachedFrameCounts& counts,
                  bool is_aeb,const ControlRequest* request,const AppliedCommand* applied,const StepMetrics& metrics) {
    const auto& ego=world.ego;
    file<<step<<','<<world.time_s<<','<<ego.pose.x<<','<<ego.pose.y<<','<<ego.pose.h<<','<<ego.speed<<','<<ego_progress_m<<','
        <<sensor_refreshed<<',';
    if(counts.available)
        file<<cached_observations.sequence<<','<<world.time_s-cached_observations.measurement_time_s<<','
            <<counts.ideal_detections<<','<<cached_observations.objects.size()<<','<<counts.dropped_detections<<',';
    else file<<",,,,,";
    const bool has_selected_target=request && request->lead_id>=0;
    if(has_selected_target) {
        file<<request->lead_id<<','; write_finite(file,request->observed_gap_m); file<<',';
        if(is_aeb) write_finite(file,request->observed_ttc_s);
        file<<',';
    } else file<<",,,";
    if(request && applied) {
        file<<1<<','<<request->desired_speed_mps<<','<<request->acceleration_mps2<<','<<applied->acceleration_mps2<<','<<applied->limited<<',';
        if(is_aeb) file<<request->aeb_active<<',';
        else file<<','<<request->close_gap_stop;
        file<<',';
    } else file<<0<<",,,,,,,";
    write_finite(file,metrics.gap_m); file<<',';
    if(metrics.target_id>=0) file<<metrics.target_id;
    file<<','; write_finite(file,metrics.ttc_s); file<<',';
    write_finite(file,metrics.minimum_distance_m); file<<','<<metrics.collision<<'\n';
}
// Frame counts of the Markov chain states, across all target chains.
struct MarkovStats {
    std::uint64_t frames=0,missing_frames=0,longest_missing_frames=0;
};
void write_summary(const fs::path& output,const RunSummary& summary,const MarkovStats& markov_stats,bool uses_markov,
                   const std::string& end_reason,double end_time,double ego_final_speed,double sensor_period) {
    auto file=open(output/"summary.json");
    file<<"{\n  \"complete\": true,\n  \"collision\": "<<(summary.collision?"true":"false")<<",\n"
        <<"  \"end_reason\": "<<json(end_reason)<<",\n  \"duration_s\": "<<end_time<<",\n"
        <<"  \"first_collision_s\": "<<event_numeric(summary.first_collision_s)<<",\n"
        <<"  \"collision_ego_speed_mps\": "<<event_numeric(summary.collision_ego_speed_mps)<<",\n"
        <<"  \"collision_relative_speed_mps\": "<<event_numeric(summary.collision_relative_speed_mps)<<",\n"
        <<"  \"minimum_distance_m\": "<<numeric(summary.minimum_distance_m)<<",\n"
        <<"  \"minimum_gap_m\": "<<numeric(summary.minimum_gap_m)<<",\n"
        <<"  \"minimum_ttc_s\": "<<numeric(summary.minimum_ttc_s)<<",\n"
        <<"  \"first_aeb_s\": "<<event_numeric(summary.first_aeb_s)<<",\n"
        <<"  \"first_deceleration_s\": "<<event_numeric(summary.first_deceleration_s)<<",\n"
        <<"  \"minimum_acceleration_mps2\": "<<summary.minimum_acceleration_mps2<<",\n"
        <<"  \"ego_final_speed_mps\": "<<ego_final_speed<<",\n"
        <<"  \"sensor_frames\": "<<summary.sensor_frames<<",\n  \"raw_detections\": "<<summary.raw_detections<<",\n"
        <<"  \"dropped_detections\": "<<summary.dropped_detections<<",\n"
        <<"  \"realized_dropout_rate\": "<<(summary.raw_detections?static_cast<double>(summary.dropped_detections)/summary.raw_detections:0)<<",\n"
        <<"  \"longest_missing_frames\": "<<summary.longest_missing_frames<<",\n"
        <<"  \"longest_missing_s\": "<<summary.longest_missing_frames*sensor_period<<",\n"
        <<"  \"model_missing_fraction\": "<<(markov_stats.frames?numeric(static_cast<double>(markov_stats.missing_frames)/markov_stats.frames):"null")<<",\n"
        <<"  \"longest_model_missing_frames\": "<<(uses_markov?std::to_string(markov_stats.longest_missing_frames):"null")<<",\n"
        <<"  \"close_gap_stop_requested\": "<<(summary.close_gap_stop_requested?"true":"false")<<"\n}\n";
}
int run(const Options& options) {
    // ===== 1. Initialization =====================================================
    fs::create_directories(options.output);
    auto scenario_id=options.source.stem().string();
    const std::string source_prefix="C_original_";
    if(scenario_id.rfind(source_prefix,0)==0) scenario_id.erase(0,source_prefix.size());
    const auto scene=prepare_scenario(options.source,options.output/"generated"/("C_"+options.controller+"_"+scenario_id+".xosc"));
    const auto original_hash=fingerprint(scene.source),road_hash=fingerprint(scene.road);
    if(options.duration>scene.duration_s+1e-8) throw std::invalid_argument("duration exceeds original scenario stop time");

    // ACC settings; its acceleration limits also bound the AEB request in arbitrate().
    AccConfig acc_config; acc_config.time_gap_s=options.time_gap;
    acc_config.set_speed_mps=options.set_speed<0 ? scene.reference.initial_speed():options.set_speed;
    acc_config.max_acceleration=scene.max_acceleration; acc_config.max_deceleration=scene.max_deceleration;
    write_config(options,scene,acc_config);

    EsminiAdapter engine(scene,options.output,options.resources,SensorConfig{},options.dt);
    EsminiAcc acc_controller(acc_config);
    std::unique_ptr<EsminiAeb> aeb_controller;
    if(options.controller=="aeb") aeb_controller=std::make_unique<EsminiAeb>(AebConfig{options.aeb_ttc_s,options.aeb_deceleration});

    DetectionDropout iid_dropout(options.dropout_p,options.seed);
    std::unique_ptr<MarkovDropout> markov_dropout;
    if(options.dropout_model=="markov") {
        std::vector<int> ids;
        for(const auto& target:engine.truth().targets) ids.push_back(target.id); // Identity only; no target truth values enter uncertainty.
        markov_dropout=std::make_unique<MarkovDropout>(options.dropout_p,options.mean_missing_s,
            options.sensor_period,options.seed,ids,options.markov_init=="stationary");
    }

    auto logs=open_run_logs(options.output,options.detailed_logs);
    EgoMotion ego; ego.state=engine.truth().ego;
    if(std::abs(ego.state.speed-scene.reference.initial_speed())>1e-6) throw std::runtime_error("Initial ego speed does not match trajectory-derived initial state");

    const auto steps_per_sensor_frame=static_cast<std::uint64_t>(std::llround(options.sensor_period/options.dt));
    PerceptionFrame cached_observations; // Latest filtered frame; reused on steps without a sensor refresh.
    CachedFrameCounts cached_counts;
    const bool is_aeb=aeb_controller!=nullptr;
    RunSummary summary;
    MarkovStats markov_stats;
    std::uint64_t missing_streak=0,step=0;
    double end_time=0; std::string end_reason="duration";

    // ===== 2. Simulation loop (one iteration per dt) =============================
    while(true) {
        // --- 2a. World truth: exact ego feedback, truth/metrics logs, collision check ---
        const auto world=engine.truth(); end_time=world.time_s;
        if(std::abs(world.time_s-static_cast<double>(step)*options.dt)>1e-7) throw std::runtime_error("Simulation clock lost synchronization");
        ego.state=world.ego; // Exact ego feedback, unaffected by uncertainty.
        if(logs.detailed) {
            write_truth(logs.truth,world.time_s,world.ego,"ego");
            for(const auto& target:world.targets) write_truth(logs.truth,world.time_s,target,"target");
        }
        const auto metrics=evaluate_truth(world); const bool engine_collision=engine.collision();
        if(logs.detailed) write_metrics(logs.metrics,world.time_s,metrics,engine_collision);
        if(metrics.collision!=engine_collision) throw std::runtime_error("Native collision flag disagrees with truth-box evaluation");
        summary.minimum_distance_m=std::min(summary.minimum_distance_m,metrics.minimum_distance_m);
        summary.minimum_gap_m=std::min(summary.minimum_gap_m,metrics.gap_m);
        summary.minimum_ttc_s=std::min(summary.minimum_ttc_s,metrics.ttc_s);

        // --- 2b. Stop conditions: first collision (only if stop_on_collision), duration, scenario end ---
        // A terminal results row records truth at this time, without a new sensor frame or command.
        const auto write_terminal_result=[&]{
            write_result(logs.results,step,world,ego.progress_m,false,cached_observations,cached_counts,is_aeb,nullptr,nullptr,metrics);
        };
        if(metrics.collision && !summary.collision) {
            summary.collision=true; summary.first_collision_s=world.time_s;
            summary.collision_ego_speed_mps=world.ego.speed;
            for(const auto& target:world.targets) if(geometry(world.ego,target).collision)
                summary.collision_relative_speed_mps=std::abs(geometry(world.ego,target).closing_speed_mps);
            if(options.stop_on_collision) { end_reason="collision"; write_terminal_result(); break; }
        }
        if(world.time_s>=options.duration-1e-8) { write_terminal_result(); break; }
        if(engine.quit()) { end_reason="scenario_stop"; write_terminal_result(); break; }

        // --- 2c. Sensor refresh and Markov/IID filtering (every sensor_period only) ---
        const bool is_sensor_frame=step%steps_per_sensor_frame==0;
        if(is_sensor_frame) {
            const auto ideal_frame=engine.sense(step/steps_per_sensor_frame);
            const auto filtered=markov_dropout?markov_dropout->apply(ideal_frame):iid_dropout.apply(ideal_frame); // EXACTLY ONCE per sensor refresh.
            if(markov_dropout) {
                if(logs.detailed) write_dropout_states(logs.dropout_states,ideal_frame,*markov_dropout);
                for(const auto& entry:markov_dropout->states()) {
                    const auto& state=entry.second;
                    ++markov_stats.frames;
                    if(state.missing) {
                        ++markov_stats.missing_frames;
                        markov_stats.longest_missing_frames=std::max(markov_stats.longest_missing_frames,state.run_length);
                    }
                }
            }
            cached_observations=filtered.frame;
            cached_counts={true,ideal_frame.objects.size(),filtered.dropped_ids.size()};
            if(logs.detailed) write_perception(logs.perception,ideal_frame,filtered);
            ++summary.sensor_frames; summary.raw_detections+=ideal_frame.objects.size(); summary.dropped_detections+=filtered.dropped_ids.size();
            if(!ideal_frame.objects.empty() && cached_observations.objects.empty()) ++missing_streak; else missing_streak=0;
            summary.longest_missing_frames=std::max(summary.longest_missing_frames,missing_streak);
        }

        // --- 2d. Controller: exact ego + cached observations -> request ---
        const ControllerInput input{world.time_s,options.dt,world.ego,cached_observations};
        const auto request=aeb_controller?aeb_controller->update(input):acc_controller.update(input);
        if(request.aeb_active && summary.first_aeb_s<0) summary.first_aeb_s=world.time_s;

        // --- 2e. Vehicle movement: actuator limits -> control log -> kinematics -> esmini ---
        const auto applied=arbitrate(request,acc_config);
        summary.close_gap_stop_requested=summary.close_gap_stop_requested||request.close_gap_stop;
        if(applied.acceleration_mps2<-.01 && summary.first_deceleration_s<0) summary.first_deceleration_s=world.time_s;
        summary.minimum_acceleration_mps2=std::min(summary.minimum_acceleration_mps2,applied.acceleration_mps2);
        if(logs.detailed) write_control(logs.control,world.time_s,cached_observations,is_sensor_frame,request,applied);
        write_result(logs.results,step,world,ego.progress_m,is_sensor_frame,cached_observations,cached_counts,is_aeb,&request,&applied,metrics);
        advance_vehicle(ego,applied,options.dt,scene.reference);
        engine.advance(ego,applied,options.dt); ++step;
    }

    // ===== 3. Results ============================================================
    if(fingerprint(scene.source)!=original_hash || fingerprint(scene.road)!=road_hash) throw std::runtime_error("Original data changed during run");
    write_summary(options.output,summary,markov_stats,markov_dropout!=nullptr,end_reason,end_time,ego.state.speed,options.sensor_period);
    std::cout<<"Completed: "<<options.output<<"\ncollision="<<summary.collision<<", min_gap="<<summary.minimum_gap_m
             <<" m, min_TTC="<<summary.minimum_ttc_s<<" s, dropped="<<summary.dropped_detections<<'/'<<summary.raw_detections<<'\n';
    return 0;
}
}
int main(int argc,char** argv) {
    try { return run(parse(argc,argv)); }
    catch(const std::exception& error) { std::cerr<<"acc_sim: "<<error.what()<<'\n'; return 1; }
}
