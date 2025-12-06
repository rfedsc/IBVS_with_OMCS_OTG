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

    // -------- 初始化为位置模式 --------
    void initialize(const std::array<double, 6>& init_position,
                    const std::array<double, 6>& init_velocity,
                    const std::array<double, 6>& init_acceleration,
                    const std::array<double, 6>& target_position,
                    const std::array<double, 6>& target_velocity,
                    const std::array<double, 6>& target_acceleration,
                    const std::array<double, 6>& max_velocity,
                    const std::array<double, 6>& max_acceleration,
                    const std::array<double, 6>& max_jerk) {

        input_->control_interface = ControlInterface::Position;

        input_->current_position = init_position;
        input_->current_velocity = init_velocity;
        input_->current_acceleration = init_acceleration;

        input_->target_position = target_position;
        input_->target_velocity = target_velocity;
        input_->target_acceleration = target_acceleration;

        input_->max_velocity = max_velocity;
        input_->max_acceleration = max_acceleration;
        input_->max_jerk = max_jerk;

        task_finished_ = false;
        has_new_target_ = false;
    }


    // -------- 更新当前位置+目标位置 + 末端速度 + 末端加速度 + 限制 --------
    void update_targets(const std::array<double, 6>& curr_pos,
                        const std::array<double, 6>& curr_vel,
                        const std::array<double, 6>& curr_acc,
                        const std::array<double, 6>& new_position, 
                        const std::array<double, 6>& new_velocity,
                        const std::array<double, 6>& new_acceleration,
                        const std::array<double, 6>& new_max_vel,
                        const std::array<double, 6>& new_max_acc,
                        const std::array<double, 6>& new_max_jerk) {

        has_new_target_ = true;
        
	new_current_position_ = curr_pos;
        new_current_velocity_ = curr_vel;
        new_current_acceleration_ = curr_acc;
        
        new_target_position_ = new_position;
        new_target_velocity_ = new_velocity;
        new_target_acceleration_ = new_acceleration;

        new_max_vel_ = new_max_vel;
        new_max_acc_ = new_max_acc;
        new_max_jerk_ = new_max_jerk;
    }

    // -------- 单周期运行 Ruckig --------
    void step() {
        if (has_new_target_) {
            //更新当前状态
            input_->current_position = new_current_position_ ;
            input_->current_velocity = new_current_velocity_;
            input_->current_acceleration = new_current_acceleration_ ;
            
            // 更新目标参数
            input_->target_position = new_target_position_;
            input_->target_velocity = new_target_velocity_;
            input_->target_acceleration = new_target_acceleration_;

            // 更新运动约束
            input_->max_velocity = new_max_vel_;
            input_->max_acceleration = new_max_acc_;
            input_->max_jerk = new_max_jerk_;

            has_new_target_ = false;
            task_finished_ = false; //
        }

        // 执行Ruckig核心计算
        last_result_ = otg_->update(*input_, *output_);

        // 传递输出到下一轮输入
        if (last_result_ == Result::Working) {
            output_->pass_to_input(*input_);
        } else if (last_result_ == Result::Finished) {
            task_finished_ = true;
        }
    }

    // -------- 返回当前状态 --------
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
    
    std::array<double, 6> new_current_position_{};
    std::array<double, 6> new_current_velocity_{};
    std::array<double, 6> new_current_acceleration_{};

    std::array<double, 6> new_target_position_{};
    std::array<double, 6> new_target_velocity_{};
    std::array<double, 6> new_target_acceleration_{};

    std::array<double, 6> new_max_vel_{};
    std::array<double, 6> new_max_acc_{};
    std::array<double, 6> new_max_jerk_{};

    bool task_finished_;
    bool has_new_target_;

    double control_dt_;
    Result last_result_ = Result::Working;
};
