from bokeh.models import HoverTool, WheelZoomTool
import json
import sys
import os
import math
def GraphGenerator(bkp, ctrl_data):
    fig_control_1_min_range = min(ctrl_data["remain_s_final"]) - 0.1
    fig_control_1_max_range = 5.0
    fig_control_1 = bkp.figure(title="remain_distance_m", x_axis_label='t(s)', y_axis_label='distance(m)', width=700, height=300, y_range=[fig_control_1_min_range,fig_control_1_max_range])
    fig1_line1 = fig_control_1.line(ctrl_data['tim_control_command'], ctrl_data['remain_s_uss'], color="black", line_dash="4 4", legend_label="remain_s_uss")
    fig1_line2 = fig_control_1.line(ctrl_data['tim_control_command'], ctrl_data['remain_s_plan'], color="blue", line_dash="4 4", legend_label="remain_s_plan")
    fig1_line3 = fig_control_1.line(ctrl_data['tim_control_command'], ctrl_data['remain_s_final'], color="red",line_width=2, alpha = 0.6, legend_label="remain_s_final")
    hover1 = HoverTool(renderers=[fig1_line3],
                        tooltips=[('remain_s_uss', '@remain_s_uss'),  ('remain_s_plan', '@remain_s_plan'),
                                ('remain_s_final', '@remain_s_final')], mode='vline')
    fig_control_1.add_tools(hover1)
    fig_control_1.toolbar.active_scroll = fig_control_1.select_one(WheelZoomTool)
    fig_control_1.legend.click_policy = 'hide'

    fig_control_2 = bkp.figure(title="velocity_mps", x_axis_label='t(s)', y_axis_label='velocity(mps)', x_range=fig_control_1.x_range, width=700, height=300, y_range=[-0.3, 1])
    fig2_line1 = fig_control_2.line(ctrl_data['tim_control_command'], ctrl_data['velocity_mps_with_sign'], color="black", alpha =  0.6, line_width=2, legend_label="v_mps")
    fig2_line2 = fig_control_2.line(ctrl_data['tim_control_command'], ctrl_data['velocity_pid_setpoint'],  color="blue", alpha= 0.6, line_width=2, legend_label="v_pid")
    fig2_line3 = fig_control_2.line(ctrl_data['tim_planning'], ctrl_data['velocity_planning'], color="red",line_width=2, alpha = 0.6, legend_label="v_plan")

    hover1 = HoverTool(renderers=[fig2_line1],
                        tooltips=[('velocity_mps_with_sign', '@velocity_mps_with_sign'), ('velocity_pid_setpoint', '@velocity_pid_setpoint')], mode='vline')
    hover2 = HoverTool(renderers=[fig2_line3],
                        tooltips=[('velocity_planning', '@velocity_planning')], mode='vline')                               
    fig_control_2.add_tools(hover1)
    fig_control_2.add_tools(hover2)
    fig_control_2.toolbar.active_scroll = fig_control_2.select_one(WheelZoomTool)
    fig_control_2.legend.click_policy = 'hide'

    fig_control_3 = bkp.figure(title="front_wheel_angle_deg", x_axis_label='t(s)', y_axis_label='angle(deg)', x_range=fig_control_1.x_range, width=700, height=300, y_range=[-40, 40])
    fig3_line1 = fig_control_3.line(ctrl_data['tim_control_command'], ctrl_data['fw_angle_deg'], color="black", alpha =  0.6, line_width=2, legend_label="fw_angle_deg")
    fig3_line2 = fig_control_3.line(ctrl_data['tim_control_command'], ctrl_data['fw_angle_max_deg'], color="blue", alpha= 0.6, line_width=2, legend_label="fw_angle_max_deg")
    fig3_line2 = fig_control_3.line(ctrl_data['tim_control_command'], ctrl_data['fw_angle_min_deg'], color="blue", alpha= 0.6, line_width=2, legend_label="fw_angle_max_deg")

    hover1 = HoverTool(renderers=[fig3_line1],
                        tooltips=[('fw_angle_deg', '@fw_angle_deg'), ('fw_angle_max_deg', '@fw_angle_max_deg'), ('fw_angle_min_deg', '@fw_angle_min_deg')], mode='vline')                            
    fig_control_3.add_tools(hover1)
    fig_control_3.toolbar.active_scroll = fig_control_3.select_one(WheelZoomTool)
    fig_control_3.legend.click_policy = 'hide'

    return fig_control_1, fig_control_2, fig_control_3


def ControlDebugGenerator(control_data, planning_data, min_t):
    data = {}

    key_list = ["tim_control_command", "remain_s_uss", "remain_s_plan", "remain_s_final","velocity_mps_without_sign", "velocity_pid_setpoint", \
    "figure1_total", "figure2_total", "figure1_line_x", "figure1_line_y", "figure2_line_y", "figure2_point_y", "figure2_point_color", \
    "figure1_point_x", "figure1_point_y", "figure1_point_color", "tim_planning", "velocity_planning", "velocity_mps_with_sign", "fw_angle_deg", \
    "fw_angle_max_deg", "fw_angle_min_deg", "fw_angle_rate", "fw_angle_max_rate", "figure3_point_y", "figure3_point_color", "figure3_total", \
    "figure3_line_y", "figure3_line_extra_x", "figure3_line_extra_y"]


    for key in key_list:
        data[key] = []

    for d in control_data:
        param_string = d[0].extra.json
        param_json = json.loads(param_string)

        data["tim_control_command"].append(d[1] - min_t)
        
        try:
            if(param_json["remain_s_uss"] > 100.0):
                data["remain_s_uss"].append(100.0)
            else:
                data["remain_s_uss"].append(param_json["remain_s_uss"])
            data["remain_s_plan"].append(param_json["remain_s_plan"])
            if(param_json["is_limited_"]):
                data["remain_s_final"].append(param_json["remain_s_uss"])
            else:
                data["remain_s_final"].append(param_json["remain_s_plan"])

        except:
            data["remain_s_uss"].append(0.0)
            data["remain_s_final"].append(0.0)
            data["remain_s_plan"].append(0.0)

        data["figure1_total"].append('uss: ' + str(data["remain_s_uss"][-1]) + '\n' + 'plan: ' + str(data["remain_s_plan"][-1]) + '\n' + 'final: ' + str(data["remain_s_final"][-1]))
        data["figure1_line_x"].append([d[1] - min_t, d[1] - min_t])
        data["figure1_line_y"].append([-1, 100])

        data["figure1_point_x"].append([d[1] - min_t, d[1] - min_t, d[1] - min_t])
        data["figure1_point_y"].append([data["remain_s_uss"][-1], data["remain_s_plan"][-1], data["remain_s_final"][-1]])
        data["figure1_point_color"].append(["black", "blue", "red"])

        try:
            data["velocity_mps_without_sign"].append(param_json["velocity_mps"])
            data["velocity_pid_setpoint"].append(param_json["velocity_pid_setpoint"])
            data["velocity_mps_with_sign"].append(param_json["format_log_lon"].split(',')[2])
        except:
            data["velocity_mps_without_sign"].append(0.0)
            data["velocity_pid_setpoint"].append(0.0)
            data["velocity_mps_with_sign"].append(0.0)


        data["figure2_total"].append('v_mps: ' + str(data["velocity_mps_with_sign"][-1]) + '\n' + 'v_pid: ' + str(data["velocity_pid_setpoint"][-1]) + '\n' + 'v_plan: null ')
        data["figure2_line_y"].append([-1, 5])
        data["figure2_point_y"].append([float(data["velocity_mps_with_sign"][-1]), data["velocity_pid_setpoint"][-1], 0.0])
        data["figure2_point_color"].append(["black", "blue", "red"])

        try:
            data["fw_angle_deg"].append(param_json["front_wheel_angle"] * 57.2957795)
            data["fw_angle_max_deg"].append(35.0)
            data["fw_angle_min_deg"].append(-35.0)
            data["fw_angle_rate"].append(0.0)
            data["fw_angle_max_rate"].append(0.0)
        except:
            data["fw_angle_deg"].append(0.0)
            data["fw_angle_max_deg"].append(35.0)
            data["fw_angle_min_deg"].append(-35.0)
            data["fw_angle_rate"].append(0.0)
            data["fw_angle_max_rate"].append(0.0)

        data["figure3_total"].append('fw_angle_deg: ' + str(data["fw_angle_deg"][-1]) + '\n' + 'fw_angle_max_deg: ' + str(data["fw_angle_max_deg"][-1]))
        data["figure3_line_y"].append([-50, 50])
        data["figure3_point_y"].append([data["fw_angle_deg"][-1], data["fw_angle_max_deg"][-1], data["fw_angle_min_deg"][-1]])
        data["figure3_point_color"].append(["black", "blue", "blue"])

    for i in range(len(data["fw_angle_deg"])):
        if(i == 0):
            data["figure3_line_extra_y"].append([-30, 30])
        elif(data["fw_angle_deg"][i] - data["fw_angle_deg"][i-1]> 0.0):
            data["figure3_line_extra_y"].append([data["fw_angle_deg"][i]-60, data["fw_angle_deg"][i]+60])
        elif(data["fw_angle_deg"][i] - data["fw_angle_deg"][i-1] < 0.0):
            data["figure3_line_extra_y"].append([data["fw_angle_deg"][i]+60, data["fw_angle_deg"][i]-60])
        else:
            data["figure3_line_extra_y"].append([data["fw_angle_deg"][i], data["fw_angle_deg"][i]])
        data["figure3_line_extra_x"].append([data["tim_control_command"][i] - 2, data["tim_control_command"][i] + 2])

    for d in planning_data:
        data["tim_planning"].append(d[1] - min_t)
        data["velocity_planning"].append(abs(d[0].trajectory.velocity.target_value))

    return data

