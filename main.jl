include("execK.jl")

# Test settings
iter_max = 10000
# 100000
# Agent Settings
w = 2
c = 2
max_speed = 0.1                     # Speed of fast agents
max_speed_fast = 0.26                # Speed of slow agents
n_particles = 40                    # Number of agents
n_neighbours = [14] #range(1, stop=40, step=1)  # Number of neighbours
width = exp10.(range(0.6, stop=2.35, length=125)) # Width of operating space
boundaries = width/2
memory = [10] #range(0, stop=50, step=2) # Memory length
n_fs = [0] #range(0, stop=40, step=2)   # Number of fast agents
sensing_radius = 1                  # Sensing range


# Behaviour Settings
rep_radius_max = 12 #12 # Maximum repulsion radius
rep_radius_min = 2  # Minimum repulsion radius
d_def = 6           # Exponential constant


# Target Settings
n_targets = 1
target_speed = 0.3  # range(0.1, stop=0.26, step=0.02)
detection_radius = 1
turn_limit = 180
movement_policy = "mix" # [ne, e, mix] (ne - non-evasive, e - pure evasive, mix - evasive then attempts to outrun when cornered)
encounter_limit = 10
jump_timer = 30
rep_radius = 1

#CSV Names
nameScore = ""
nameProportion = ""
nameDensity = ""

# More
k_diff = 5 # Used if we need a way to modify k with a static number, often 5, for more/less exploring and incitateSpeed.
cond = "none"




# You can use the following : 

# "lessTracking", "moreTracking", "lessExploring", "moreExploring", "SlowFleeing", "FleeHighDens", "PerfOnlyTs" (timestamp), "PerfOnlyMs" (movespeed), "PerfOnlyNr" (distance norm)
# "incitateSpeed", "NewRepulsion", "RatioExplo", "RatioDens"

original_values = [1000,2,2,0.1,0.26,50,[15], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.65, length=125)), #  8
    exp10.(range(0.6, stop=2.65, length=125))/2, #  9
    [20],[0],1, # End sensing Radius  10,11,12
    6,2,6, # Behaviour settings   13,14,15
    1,0.15,1,180,"ne",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "y/Parallel_Score.csv", "y/Parallel_Proportion.csv"] # CSV names    24,25



updated_values_1 = [100000,2,2,0.1,0.26,20,[6], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[20],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "setCost/Score_0_20_100.csv", "setCost/Proportion_0_20_100.csv", "setCost/Density_0_20_100.csv",# CSV names    24,25
    5, "none"] # k_difference and cond

updated_values_2 = [100000,2,2,0.1,0.26,40,[15], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[10],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "equilibrium/Score_40_8_Density_35_100.csv", "equilibrium/Proportion_40_8_Density_35_100.csv", "equilibrium/Density_40_8_Density_35_100.csv",# CSV names    24,25
    5, "moreExploring"] # k_difference and cond

updated_values_3 = [100000,2,2,0.1,0.26,40,[10], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[10],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "dynamicK/Parallel_Score_40_10_k10_less_exploring_5_100_000.csv", "dynamicK/Parallel_Proportion_40_10_k10_less_exploring_5_100_000.csv", "dynamicK/Parallel_Density_40_10_k10_less_exploring_5_100_000.csv",# CSV names    24,25
    5, "lessExploring"] # k_difference and cond

updated_values_4 = [100000,2,2,0.1,0.26,40,[15], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[10],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "dynamicK/Parallel_Score_40_10_k15_less_exploring_5_100_000.csv", "dynamicK/Parallel_Proportion_40_10_k15_less_exploring_5_100_000.csv", "dynamicK/Parallel_Density_40_10_k15_less_exploring_5_100_000.csv",# CSV names    24,25
    5, "lessExploring"] # k_difference and cond



updated_values_5 = [100000,2,2,0.1,0.26,60,[14], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2)[70], #  9
    [10],[0],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "wave/Score_60_k14_d70_100.csv", "wave/Proportion_60_k14_d70_100.csv", "wave/Density_60_k14_d70_100.csv",# CSV names    24,25
    5, "none"] # k_difference and cond

updated_values_6 = [100000,2,2,0.1,0.26,40,[17], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[8],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "regression/Parallel_Score_40_8_k17_100_000.csv", "regression/Parallel_Proportion_40_8_k17_100_000.csv", "regression/Parallel_Density_40_8_k17_100_000.csv",# CSV names    24,25
    5, "none"] # k_difference and cond


updated_values_7 = [100000,2,2,0.1,0.26,40,[20], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[8],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "regression/Parallel_Score_40_8_k20_100_000.csv", "regression/Parallel_Proportion_40_8_k20_100_000.csv", "regression/Parallel_Density_40_8_k20_100_000.csv",# CSV names    24,25
    5, "none"] # k_difference and cond

updated_values_8 = [100000,2,2,0.1,0.26,40,[25], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[8],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "regression/Parallel_Score_40_8_k25_100_000.csv", "regression/Parallel_Proportion_40_8_k25_100_000.csv", "regression/Parallel_Density_40_8_k25_100_000.csv",# CSV names    24,25
    5, "none"] # k_difference and cond

updated_values_9 = [10000,2,2,0.1,0.26,40,[30], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[32],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "classicModifK2/Parallel_Score_40_32_k30_10_000.csv", "classicModifK2/Parallel_Proportion_40_32_k30_10_000.csv", "classicModifK2/Parallel_Density_40_32_k30_10_000.csv",# CSV names    24,25
    10, "none"] # k_difference and cond

updated_values_10 = [10000,2,2,0.1,0.26,40,[30], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    (exp10.(range(0.6, stop=2.35, length=125))/2), #  9
    [10],[24],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "classicModifK2/Parallel_Score_40_24_k30_10_000.csv", "classicModifK2/Parallel_Proportion_40_24_k30_10_000.csv", "classicModifK2/Parallel_Density_40_24_k30_10_000.csv",# CSV names    24,25
    5, "none"] # k_difference and cond





width = exp10.(range(0.6, stop=2.35, length=125)) # Width of operating space
boundaries = width/2


tableau = [ 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  7,  7, # Gotten from regressions.ipynb
7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
10 ,10, 10, 10, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25,
25, 25, 25, 25, 25, 25, 25, 25, 25, 25 ,25 ,25 ,25 ,25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25,
25, 25, 25, 25, 25, 25, 25, 25, 25, 25 ,25 ,25 ,25 ,25 ,25 ,25, 25, 25, 25, 25, 25, 25, 20, 20,
20, 20, 20, 20, 20]

tableau_tuples = zip(boundaries, tableau)

updated_values_11 = [100000,2,2,0.1,0.26,40,[10], # Start sensing Radius   1,2,3,4,5,6,7
    exp10.(range(0.6, stop=2.35, length=125)), #  8
    tableau_tuples, #  9
    [10],[8],1, # End sensing Radius  10,11,12
    12,2,6, # Behaviour settings   13,14,15
    1,0.3,1,180,"mix",10,30,1, # Target Settings   16,17,18,19,20,21,22,23
    "regression/Parallel_Score_40_8_k_DYNAMIC_100_000.csv", "regression/Parallel_Proportion_40_8_k_DYNAMIC_100_000.csv", "regression/Parallel_Density_40_8_k_DYNAMIC_100_000.csv",# CSV names    24,25
    5, "none"] # k_difference and cond



    

arrays = Dict([("parameters_array_0",updated_values_1),
    ("parameters_array_1", updated_values_2),
    ("parameters_array_2", updated_values_3),
    ("parameters_array_3", updated_values_4),
    ("parameters_array_4"),
    ("parameters_array_5"),
    ("parameters_array_6"),
    ("parameters_array_7"),
    ("parameters_array_8"),
    ("parameters_array_9")])

for i in 0:1:10 #Modify 9 if adding new arrays in the dict.
    if get(arrays, "parameters_array_"*string(i), 0) != 0
        main(get(arrays, "parameters_array_"*string(i), 0))
    end
end
