#pragma once
#include <ruckig/ruckig.hpp>
#include <array>
#include <memory>

using namespace ruckig;

class OTGManager {
public:
    OTGManager(double control_dt = 0.01)
        : control_dt_(control_dt) {

        otg_ = std::make_unique<Ruckig<6>>(control_dt_);
        input_ = std::make_unique<InputParameter<6>>();
        output_ = std::make_unique<OutputParameter<6>>();

        task_finished_ = false;
        has_new_target_ = false;
    }

    void initialize(const std::array<double, 6>& init_position,
                    const std::array<double, 6>& init_velocity,
                    const std::array<double, 6>& init_acceleration,
                    const std::array<double, 6>& target_velocity,
                    const std::array<double, 6>& target_acceleration,
                    const std::array<double, 6>& max_acceleration,
                    const std::array<double, 6>& max_jerk,
                    double minimum_duration_time) {

        input_->control_interface = ControlInterface::Velocity;

        input_->current_position = init_position;
        input_->current_velocity = init_velocity;
        input_->current_acceleration = init_acceleration;

        input_->target_velocity = target_velocity;
        input_->target_acceleration = target_acceleration;

        input_->max_acceleration = max_acceleration;
        input_->max_jerk = max_jerk;

        input_->minimum_duration = minimum_duration_time;

        task_finished_ = false;
        has_new_target_ = false;
    }

    void update_velocity(const std::array<double, 6>& new_velocity) {
        has_new_target_ = true;
        new_velocity_ = new_velocity;
        new_acceleration_.fill(0.0);
    }

    void update_velocity_acc(const std::array<double, 6>& new_velocity,
                             const std::array<double, 6>& new_acc) {
        has_new_target_ = true;
        new_velocity_ = new_velocity;
        new_acceleration_ = new_acc;
    }

    void update_targets(const std::array<double, 6>& new_velocity,
                        const std::array<double, 6>& new_acceleration,
                        const std::array<double, 6>& new_max_acc,
                        const std::array<double, 6>& new_max_jerk,
                        double new_min_dur) {

        has_new_target_ = true;

        new_velocity_ = new_velocity;
        new_acceleration_ = new_acceleration;
        new_max_acc_ = new_max_acc;
        new_max_jerk_ = new_max_jerk;
        new_minimum_duration_time_ = new_min_dur;
    }

    void step() {
        if (has_new_target_) {
            input_->target_velocity = new_velocity_;
            input_->target_acceleration = new_acceleration_;

            // 如果用户更新了限制，也应用
            if (!new_max_acc_.empty()) {
                input_->max_acceleration = new_max_acc_;
                input_->max_jerk = new_max_jerk_;
                input_->minimum_duration = new_minimum_duration_time_;
            }

            has_new_target_ = false;
        }

        last_result_ = otg_->update(*input_, *output_);

        if (last_result_ == Result::Working) {
            output_->pass_to_input(*input_);
        } else if (last_result_ == Result::Finished) {
            task_finished_ = true;
        }
    }

    void get_current_state(std::array<double, 6>& pos,
                           std::array<double, 6>& vel,
                           std::array<double, 6>& acc,
                           std::array<double, 6>& jerk) {
        pos = output_->new_position;
        vel = output_->new_velocity;
        acc = output_->new_acceleration;
        jerk = output_->new_jerk;
    }

    bool is_finished() const { return task_finished_; }
    Result last_result() const { return last_result_; }

private:
    std::unique_ptr<Ruckig<6>> otg_;
    std::unique_ptr<InputParameter<6>> input_;
    std::unique_ptr<OutputParameter<6>> output_;

    std::array<double, 6> new_velocity_{};
    std::array<double, 6> new_acceleration_{};
    std::array<double, 6> new_max_acc_{};
    std::array<double, 6> new_max_jerk_{};
    double new_minimum_duration_time_{0.0};

    bool task_finished_;
    bool has_new_target_;

    double control_dt_;
    Result last_result_ = Result::Working;
};

