classdef ServoMotorModel < handle
    properties
        prev_delta
        prev_rate
        t_step
        delta_dot_max
        rate_lpf_gain
        motor_num
    end

    methods
        function obj = ServoMotorModel(motor_num, max_rate, t_step)
            obj.motor_num     = motor_num;
            obj.prev_delta    = NaN(motor_num,1);
            obj.prev_rate     = zeros(motor_num,1);
            obj.t_step        = t_step;
            obj.delta_dot_max = deg2rad(max_rate);

            tau = 0.01;  % time constant
            obj.rate_lpf_gain = exp(-obj.t_step / tau);
        end

        function delta_rad = forward(obj, ref_rate)
            if any(isnan(obj.prev_delta))
                obj.prev_delta = zeros(obj.motor_num,1);
            end

            % 속도 명령에 LPF
            rate_f = obj.prev_rate * obj.rate_lpf_gain + ...
                     ref_rate     * (1 - obj.rate_lpf_gain);

            % 레이트 리밋
            rate_f = max(min(rate_f,  obj.delta_dot_max), ...
                         -obj.delta_dot_max);

            % 적분
            delta_rad      = obj.prev_delta + rate_f * obj.t_step;
            obj.prev_delta = delta_rad;
            obj.prev_rate  = rate_f;
        end
    end
end
