// Studio perception noise for the isolated HiDrive esmini 3.8.1 build.
//
// Public esmini code only: the supplied controller is not modified. Around each
// Step() of a controller attached to the observer, every other object is moved
// to its perceived centre (true centre range + Studio OU range error, along the
// ego-centre line of sight) and restored right afterwards. Collision detection,
// the CSV logger and the replay therefore keep the true state. Without
// --perception_noise nothing changes.
//
// Noise file (tab separated):
//   observer <name> | zero_range_m <v> | reference_range_m <v> |
//   reference_mae_pct <v> | dt_s <v> | hide_distance_m <v> | log <path> |
//   z <object name> <z at step 0> <z at step 1> ...
// Optional fixed errors, neutral when absent:
//   scale <v> | bias_m <v> | wrong_scale <v> | wrong_bias_m <v> |
//   missed <object name> <0/1 at step 0> ... | wrong <object name> <0/1 at step 0> ... |
//   predict_when_lost <0/1> | freeze_motion_when_lost <0/1> | live_observer_when_lost <0/1>
// The unit OU samples z come from Studio (models.CorrelatedRangeNoise); the
// distance-dependent amplitude is applied here on the live true range, exactly
// as models.range_error_parameters does. The measured range is
//   (scale - 1) * range + bias + sigma * z + range,
// using wrong_scale and wrong_bias while the object is flagged wrong. A negative
// measured range or a missed step is invalid: the object is moved hide_distance_m
// behind the observer instead.
// With predict_when_lost, a missed step after the object has been seen reports
//   last measured range - (observer speed - object speed, both at that step) * elapsed time,
// no lower than the two boxes touching. Only the position changes, as for every other error,
// unless freeze_motion_when_lost also holds the object's speed, velocity and acceleration at
// that step during the controller step. With live_observer_when_lost the range instead steps on
// by (observer speed now - object speed at the last seen step) * step, so only the object's speed
// is held in the distance as well.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "CommonMini.hpp"
#include "Entities.hpp"
#include "logger.hpp"

namespace studio
{
    class PerceptionNoise
    {
    public:
        static PerceptionNoise& Inst()
        {
            static PerceptionNoise instance;
            return instance;
        }

        // Move the objects seen by the controller of `observer` to their perceived poses.
        void Begin(scenarioengine::Object* observer, std::vector<scenarioengine::Object*>& objects, double time)
        {
            Load();
            if (!enabled_ || observer == nullptr || observer->GetName() != observer_)
            {
                return;
            }
            long   step = std::lround(time / dt_);
            double ex, ey;
            Center(observer, ex, ey);
            for (scenarioengine::Object* obj : objects)
            {
                if (obj == nullptr || obj == observer)
                {
                    continue;
                }
                auto sequence = z_.find(obj->GetName());
                if (sequence == z_.end())
                {
                    Fail("no noise sequence for object " + obj->GetName());
                }
                if (step < 0 || step >= static_cast<long>(sequence->second.size()))
                {
                    Fail("noise sequence too short at t=" + std::to_string(time));
                }
                double z = sequence->second[static_cast<size_t>(step)];
                double tx, ty;
                Center(obj, tx, ty);
                double dx = tx - ex, dy = ty - ey;
                double range = std::hypot(dx, dy);
                double mae     = reference_ * pct_ / 100.0 * std::max(0.0, range - zero_) / (reference_ - zero_);
                double sigma   = mae * std::sqrt(kPi / 2.0);
                bool   correct = !Flag(wrong_, obj->GetName(), step);
                bool   seen    = !Flag(missed_, obj->GetName(), step);
                double mean    = ((correct ? scale_ : wrong_scale_) - 1.0) * range + (correct ? bias_ : wrong_bias_);
                double error   = mean + sigma * z;
                double raw     = range + error;
                bool   predicted = false;
                if (predict_ && seen && raw >= 0.0)
                {
                    last_seen_[obj->GetName()] = LastSeen{raw,
                                                          observer->GetSpeed() - obj->GetSpeed(),
                                                          time,
                                                          obj->GetSpeed(),
                                                          obj->pos_.GetVelX(),
                                                          obj->pos_.GetVelY(),
                                                          obj->pos_.GetVelZ(),
                                                          obj->pos_.GetAccX(),
                                                          obj->pos_.GetAccY(),
                                                          obj->pos_.GetAccZ()};
                }
                else if (predict_ && !seen)
                {
                    auto last = last_seen_.find(obj->GetName());
                    if (last != last_seen_.end())
                    {
                        double contact = (observer->boundingbox_.dimensions_.length_ + obj->boundingbox_.dimensions_.length_) / 2.0;
                        double distance;
                        if (live_observer_)
                        {
                            // The held range stays unclamped: only the reported distance stops at contact.
                            last->second.range -= (observer->GetSpeed() - last->second.speed) * (time - last->second.time);
                            last->second.time = time;
                            distance          = last->second.range;
                        }
                        else
                        {
                            distance = last->second.range - last->second.closing * (time - last->second.time);
                        }
                        raw            = std::max(contact, distance);
                        error          = raw - range;
                        predicted      = true;
                    }
                }
                bool   valid   = (seen || predicted) && raw >= 0.0;
                double ox, oy;
                if (valid)
                {
                    double scale = range > 1e-12 ? raw / range : 0.0;
                    ox           = ex + dx * scale;
                    oy           = ey + dy * scale;
                }
                else
                {
                    double h = observer->pos_.GetH();
                    ox       = ex - hide_ * std::cos(h);
                    oy       = ey - hide_ * std::sin(h);
                }
                saved_.push_back(Saved{obj, obj->pos_, obj->speed_});
                obj->pos_.SetInertiaPos(obj->pos_.GetX() + ox - tx, obj->pos_.GetY() + oy - ty, obj->pos_.GetH());
                if (predicted && freeze_)
                {
                    const LastSeen& last = last_seen_.at(obj->GetName());
                    obj->speed_          = last.speed;
                    obj->pos_.SetVel(last.vx, last.vy, last.vz);
                    obj->pos_.SetAcc(last.ax, last.ay, last.az);
                }
                log_ << time << '\t' << obj->GetName() << '\t' << range << '\t' << sigma << '\t' << z << '\t' << error << '\t' << raw
                     << '\t' << (valid ? 1 : 0) << '\t' << ox << '\t' << oy << '\t' << mean << '\t' << (seen ? 1 : 0) << '\t'
                     << (correct ? 1 : 0) << '\t' << (predicted ? 1 : 0) << '\t' << obj->speed_ << '\t' << obj->pos_.GetVelX() << '\t'
                     << obj->pos_.GetVelY() << '\t' << obj->pos_.GetAccX() << '\t' << obj->pos_.GetAccY() << '\n';
            }
        }

        // Restore the true poses (esmini 3.x keeps the truth in Object::pos_ directly).
        void End()
        {
            for (auto& item : saved_)
            {
                item.object->pos_.Duplicate(item.pos);
                item.object->speed_ = item.speed;
                if (item.object->pos_.GetRoute() != nullptr)
                {
                    item.object->pos_.CalcRoutePosition();
                }
            }
            if (!saved_.empty())
            {
                log_.flush();
            }
            saved_.clear();
        }

    private:
        static constexpr double kPi = 3.14159265358979323846;

        struct LastSeen
        {
            double range, closing, time, speed, vx, vy, vz, ax, ay, az;
        };

        struct Saved
        {
            scenarioengine::Object* object;
            roadmanager::Position   pos;
            double                  speed;
        };

        bool                                         loaded_  = false;
        bool                                         enabled_ = false;
        bool                                         predict_ = false;
        bool                                         freeze_  = false;
        bool                                         live_observer_ = false;
        std::map<std::string, LastSeen>              last_seen_;
        std::string                                  observer_;
        double                                       zero_ = 0.0, reference_ = 0.0, pct_ = 0.0, dt_ = 0.0, hide_ = 1000.0;
        double                                       scale_ = 1.0, bias_ = 0.0, wrong_scale_ = 1.0, wrong_bias_ = 0.0;
        std::map<std::string, std::vector<double>>   z_, missed_, wrong_;
        std::ofstream                                log_;
        std::vector<Saved>                           saved_;

        static void Fail(const std::string& message)
        {
            LOG_ERROR("perception_noise: {}", message);
            std::fprintf(stderr, "perception_noise: %s\n", message.c_str());
            std::exit(3);
        }

        static bool Flag(const std::map<std::string, std::vector<double>>& flags, const std::string& name, long step)
        {
            auto sequence = flags.find(name);
            if (sequence == flags.end())
            {
                return false;
            }
            if (step < 0 || step >= static_cast<long>(sequence->second.size()))
            {
                Fail("flag sequence too short for object " + name);
            }
            return sequence->second[static_cast<size_t>(step)] != 0.0;
        }

        static void Center(scenarioengine::Object* obj, double& x, double& y)
        {
            double h = obj->pos_.GetH();
            double cx = obj->boundingbox_.center_.x_, cy = obj->boundingbox_.center_.y_;
            x         = obj->pos_.GetX() + std::cos(h) * cx - std::sin(h) * cy;
            y         = obj->pos_.GetY() + std::sin(h) * cx + std::cos(h) * cy;
        }

        void Load()
        {
            if (loaded_)
            {
                return;
            }
            loaded_         = true;
            SE_Options& opt = SE_Env::Inst().GetOptions();
            if (!opt.GetOptionSet("perception_noise"))
            {
                return;
            }
            std::ifstream in(opt.GetOptionValue("perception_noise"));
            if (!in)
            {
                Fail("cannot open " + opt.GetOptionValue("perception_noise"));
            }
            std::string line;
            while (std::getline(in, line))
            {
                if (!line.empty() && line.back() == '\r')
                {
                    line.pop_back();
                }
                std::vector<std::string> cells;
                std::stringstream        stream(line);
                std::string              cell;
                while (std::getline(stream, cell, '\t'))
                {
                    cells.push_back(cell);
                }
                if (cells.size() < 2)
                {
                    continue;
                }
                const std::string& key = cells[0];
                if (key == "observer")
                    observer_ = cells[1];
                else if (key == "zero_range_m")
                    zero_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "reference_range_m")
                    reference_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "reference_mae_pct")
                    pct_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "dt_s")
                    dt_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "hide_distance_m")
                    hide_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "log")
                    log_.open(cells[1]);
                else if (key == "scale")
                    scale_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "bias_m")
                    bias_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "wrong_scale")
                    wrong_scale_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "wrong_bias_m")
                    wrong_bias_ = std::strtod(cells[1].c_str(), nullptr);
                else if (key == "predict_when_lost")
                    predict_ = std::strtod(cells[1].c_str(), nullptr) != 0.0;
                else if (key == "freeze_motion_when_lost")
                    freeze_ = std::strtod(cells[1].c_str(), nullptr) != 0.0;
                else if (key == "live_observer_when_lost")
                    live_observer_ = std::strtod(cells[1].c_str(), nullptr) != 0.0;
                else if (key == "z" || key == "missed" || key == "wrong")
                {
                    std::vector<double>& values = (key == "z" ? z_ : key == "missed" ? missed_ : wrong_)[cells.at(1)];
                    for (size_t i = 2; i < cells.size(); i++)
                    {
                        values.push_back(std::strtod(cells[i].c_str(), nullptr));
                    }
                }
                else
                    Fail("unknown key " + key);
            }
            if (observer_.empty() || !(reference_ > zero_) || !(dt_ > 0.0) || !(hide_ > 0.0) || !log_.is_open())
            {
                Fail("incomplete noise file");
            }
            log_ << std::setprecision(17)
                 << "time_s\tobject_name\ttrue_range_m\tsigma_m\tz\terror_m\traw_range_m\tvalid\tobserved_center_x_m\tobserved_center_y_m"
                    "\tmean_error_m\tdetected\tclass_correct\tpredicted\tobject_speed_mps\tobject_vx_mps\tobject_vy_mps\tobject_ax_mps2\tobject_ay_mps2\n";
            enabled_ = true;
        }
    };
}  // namespace studio
