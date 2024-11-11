"""This function calculates the energy consumption of UGV in a single period based on a particular UGV route. This
function takes UGV energy consumption as a constraint and based on the returned energy consumption value, a penalty is added
to the UAV routes if the UGV's energy consumption exceeds its capacity limit. This will discourage that particular UAV-UGV solution."""


def ugv_power(nw_wait_time, se_wait_time, velocity, depotB_to_nw_velocity, nw_stop_tw_1, ugv_travel_time, return_time_1, return_time_2):
    ugv_travel_time -= 926
    power_consumption_till_nw = (464.8*depotB_to_nw_velocity + 356.3)*nw_stop_tw_1
    nw_wait_power = 200*(nw_wait_time+926) # considering the waiting time of UGV on the stop
    power_consumption_ugv_travel_between_stops = (464.8*velocity + 356.3)*ugv_travel_time
    se_wait_power = 200*(se_wait_time+926)
    return_power_consumption = (464.8*velocity + 356.3)*return_time_1 + (464.8*depotB_to_nw_velocity + 356.3)*return_time_2

    total_consumption = power_consumption_till_nw + nw_wait_power + power_consumption_ugv_travel_between_stops + \
                        se_wait_power + return_power_consumption
    return total_consumption

# ugv_consumption = ugv_power(18*60, 41*60, 14/3.281, 15/3.281, 2181, 4461, 1510, 1780)
# print(ugv_consumption)
