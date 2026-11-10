// Perception bridge for esmini 3.8.1. Public esmini code only; controllers are not modified.
//
// With --perception_bridge <observer>, before each Step() of the controller attached to the observer,
// the bridge writes the true state of every object to stdout and reads one reply line from stdin:
// how that controller should perceive each other object. It moves the objects there for the
// controller step and restores the true state right afterwards, so collision detection, the CSV
// logger and the replay keep the truth. All error logic lives in Python (pipeline/errors); this file
// only moves objects. Without --perception_bridge nothing changes.
//
// One line each way per controller step, tab separated, numbers as C99 hex floats (exact):
//   esmini -> Python   @@PB S <time> <count>, then per object, observer first:
//                      <name> <cx> <cy> <z> <h> <speed> <vx> <vy> <vz> <ax> <ay> <az> <length> <width> <height> <range>
//                      (cx, cy: centre of the bounding box; range: centre to centre from the observer)
//   Python -> esmini   @@PB R, then per other object, in the same order:
//                      <flags> <range> <x> <y> <h> <speed> <vx> <vy> <vz> <ax> <ay> <az> <length> <width>
//   flags: 1 seen, 2 place the centre at x, y (otherwise at range along the line of sight from the observer),
//          4 use heading h, 8 use speed, velocity and acceleration, 16 use length and width.
// An object that is not seen is moved kHideM behind the observer.
#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "CommonMini.hpp"
#include "Entities.hpp"
#include "logger.hpp"

namespace studio
{
    class PerceptionBridge
    {
    public:
        static PerceptionBridge& Inst()
        {
            static PerceptionBridge instance;
            return instance;
        }

        void Begin(scenarioengine::Object* observer, std::vector<scenarioengine::Object*>& objects, double time)
        {
            Load();
            if (!enabled_ || observer == nullptr || observer->GetName() != observer_)
            {
                return;
            }
            double ex, ey;
            Center(observer, ex, ey);
            std::vector<Target> targets;
            for (scenarioengine::Object* obj : objects)
            {
                if (obj == nullptr || obj == observer)
                {
                    continue;
                }
                Target target{obj, 0.0, 0.0, 0.0};
                Center(obj, target.tx, target.ty);
                target.range = std::hypot(target.tx - ex, target.ty - ey);
                targets.push_back(target);
            }

            std::string message = "@@PB\tS\t" + Hex(time) + "\t" + std::to_string(targets.size() + 1);
            Describe(message, observer, ex, ey, 0.0);
            for (const Target& target : targets)
            {
                Describe(message, target.object, target.tx, target.ty, target.range);
            }
            message += '\n';
            std::fwrite(message.data(), 1, message.size(), stdout);
            std::fflush(stdout);

            std::string reply;
            if (!std::getline(std::cin, reply))
            {
                Fail("no reply from Python");
            }
            if (!reply.empty() && reply.back() == '\r')
            {
                reply.pop_back();
            }
            std::vector<std::string> cells = Split(reply);
            if (cells.size() != 2 + targets.size() * kReplyFields || cells[0] != "@@PB" || cells[1] != "R")
            {
                Fail("malformed reply at t=" + std::to_string(time));
            }
            for (size_t i = 0; i < targets.size(); i++)
            {
                double v[kReplyFields];
                for (size_t k = 0; k < kReplyFields; k++)
                {
                    v[k] = std::strtod(cells[2 + i * kReplyFields + k].c_str(), nullptr);
                }
                Apply(observer, targets[i], ex, ey, static_cast<long>(v[0]), v);
            }
        }

        // Restore the true state (esmini 3.x keeps the truth in Object::pos_ directly).
        void End()
        {
            for (auto& item : saved_)
            {
                item.object->pos_.Duplicate(item.pos);
                item.object->speed_                           = item.speed;
                item.object->boundingbox_.dimensions_.length_ = item.length;
                item.object->boundingbox_.dimensions_.width_  = item.width;
                if (item.object->pos_.GetRoute() != nullptr)
                {
                    item.object->pos_.CalcRoutePosition();
                }
            }
            saved_.clear();
        }

    private:
        static constexpr size_t kReplyFields = 14;
        static constexpr double kHideM       = 1000.0;
        enum Flag : long
        {
            SEEN     = 1,
            POSITION = 2,
            HEADING  = 4,
            MOTION   = 8,
            SIZE     = 16
        };

        struct Target
        {
            scenarioengine::Object* object;
            double                  tx, ty, range;
        };

        struct Saved
        {
            scenarioengine::Object* object;
            roadmanager::Position   pos;
            double                  speed, length, width;
        };

        bool               loaded_  = false;
        bool               enabled_ = false;
        std::string        observer_;
        std::vector<Saved> saved_;

        static void Fail(const std::string& message)
        {
            LOG_ERROR("perception_bridge: {}", message);
            std::fprintf(stderr, "perception_bridge: %s\n", message.c_str());
            std::exit(3);
        }

        static std::string Hex(double value)
        {
            char buffer[64];
            std::snprintf(buffer, sizeof(buffer), "%a", value);
            return buffer;
        }

        static std::vector<std::string> Split(const std::string& line)
        {
            std::vector<std::string> cells;
            size_t                   start = 0;
            while (true)
            {
                size_t tab = line.find('\t', start);
                cells.push_back(line.substr(start, tab == std::string::npos ? std::string::npos : tab - start));
                if (tab == std::string::npos)
                {
                    return cells;
                }
                start = tab + 1;
            }
        }

        static void Center(scenarioengine::Object* obj, double& x, double& y)
        {
            double h = obj->pos_.GetH();
            double cx = obj->boundingbox_.center_.x_, cy = obj->boundingbox_.center_.y_;
            x         = obj->pos_.GetX() + std::cos(h) * cx - std::sin(h) * cy;
            y         = obj->pos_.GetY() + std::sin(h) * cx + std::cos(h) * cy;
        }

        static void Describe(std::string& message, scenarioengine::Object* obj, double cx, double cy, double range)
        {
            const roadmanager::Position& p = obj->pos_;
            const auto&                  d = obj->boundingbox_.dimensions_;
            message += "\t" + obj->GetName();
            for (double value : {cx,
                                 cy,
                                 p.GetZ(),
                                 p.GetH(),
                                 obj->GetSpeed(),
                                 p.GetVelX(),
                                 p.GetVelY(),
                                 p.GetVelZ(),
                                 p.GetAccX(),
                                 p.GetAccY(),
                                 p.GetAccZ(),
                                 d.length_,
                                 d.width_,
                                 d.height_,
                                 range})
            {
                message += "\t" + Hex(value);
            }
        }

        void Apply(scenarioengine::Object* observer, const Target& target, double ex, double ey, long flags, const double* v)
        {
            scenarioengine::Object* obj = target.object;
            double                  ox, oy;
            if (!(flags & SEEN))
            {
                double h = observer->pos_.GetH();
                ox       = ex - kHideM * std::cos(h);
                oy       = ey - kHideM * std::sin(h);
            }
            else if (flags & POSITION)
            {
                ox = v[2];
                oy = v[3];
            }
            else
            {
                double scale = target.range > 1e-12 ? v[1] / target.range : 0.0;
                ox           = ex + (target.tx - ex) * scale;
                oy           = ey + (target.ty - ey) * scale;
            }
            saved_.push_back(Saved{obj, obj->pos_, obj->speed_, obj->boundingbox_.dimensions_.length_, obj->boundingbox_.dimensions_.width_});
            if (flags & HEADING)
            {
                // Turn about the box centre: the reference point moves with the new heading.
                double h = v[4], cx = obj->boundingbox_.center_.x_, cy = obj->boundingbox_.center_.y_;
                obj->pos_.SetInertiaPos(ox - (std::cos(h) * cx - std::sin(h) * cy), oy - (std::sin(h) * cx + std::cos(h) * cy), h);
            }
            else
            {
                obj->pos_.SetInertiaPos(obj->pos_.GetX() + ox - target.tx, obj->pos_.GetY() + oy - target.ty, obj->pos_.GetH());
            }
            if (flags & MOTION)
            {
                obj->speed_ = v[5];
                obj->pos_.SetVel(v[6], v[7], v[8]);
                obj->pos_.SetAcc(v[9], v[10], v[11]);
            }
            if (flags & SIZE)
            {
                obj->boundingbox_.dimensions_.length_ = v[12];
                obj->boundingbox_.dimensions_.width_  = v[13];
            }
        }

        void Load()
        {
            if (loaded_)
            {
                return;
            }
            loaded_         = true;
            SE_Options& opt = SE_Env::Inst().GetOptions();
            if (!opt.GetOptionSet("perception_bridge"))
            {
                return;
            }
            observer_ = opt.GetOptionValue("perception_bridge");
            if (observer_.empty())
            {
                Fail("--perception_bridge needs the name of the observer");
            }
            enabled_ = true;
        }
    };
}  // namespace studio
