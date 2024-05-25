using Distributions, Distances, Parameters, LinearAlgebra, Random, Statistics
using DelimitedFiles, CSV, DataFrames, LightGraphs, SimpleWeightedGraphs
using PyCall, BenchmarkTools, Base.Threads


####################################################################################################################################
#                                                                                                                                  #
#                                                                                                                                  #
#                                                                                                                                  #
#                                                      Agent                                                                       #
#                                                                                                                                  #
#                                                                                                                                  #
####################################################################################################################################


@with_kw mutable struct Particle
    # User defined parameters
    name::Int
    k::Int
    oldk::Int
    max_speed::Float64
    boundary::Float64
    memory::Int
    sensing_range::Float64
    w::Float64
    c::Float64
    rep_radius_max::Float64
    rep_radius_min::Float64
    rep_radius = rep_radius_min
    d::Float64

    # State parameters
    position::Vector = [rand(Uniform(-boundary, boundary)),
                        rand(Uniform(-boundary, boundary))]
    velocity::Vector = [0., 0.]
    waypoint = [rand(Uniform(-boundary, boundary)),
                rand(Uniform(-boundary, boundary))]

    timer = 0
    mode = "explore"
    density = 0

    # Own target information
    target_pos = []
    timestamp = 0

    # Target information from neighbours
    n_target_pos = []
    n_timestamp = 0

    # Used information
    attraction_point = []
    used_timestamp = 0

    # Movement Velocities
    attract_vel = [0., 0.]
    repel_vel = [0., 0.]

    # new
    fleeing = 0
end

# Agent functions
function update_target_info!(self::Particle, target_list::Array)
    for target in target_list
        # If directly detecting target
        if norm(self.position - target.position) <= self.sensing_range
            self.mode = "track"
            self.attraction_point = self.target_pos = target.position
            self.used_timestamp = self.timestamp = self.timer
            break
        # If own memory time out, clear memory
        elseif self.target_pos != [] && self.timer > (self.timestamp +
            self.memory)
            self.mode = "explore"
            self.target_pos = []
            self.timestamp = 0
        end
    end
end

function get_neighbour_info(self::Particle, agent_list::Array, k_difference::Int, cond::String)

    if self.k == 0
        return [],  [],  [], []
    end

    theK = self.k

    stored_neighbours = 0
    n_target_pos = []
    n_times = zeros(theK)
    n_distances = zeros(theK)
    n_positions = []
    modes = []
    modes2 = []
    density_distances = []
    n = 6   # Number of nearest neighbours to consider when calculating local density

    for agent in agent_list
        if agent.name == self.name
            continue
        
        else 
            distance = norm(self.position - agent.position)

            # This block to gather distances between n neighbours to calculate local density
            if length(density_distances) < n
                push!(density_distances, distance)
            elseif distance < maximum(density_distances)
                density_distances[argmax(density_distances)] = distance
            end

            
            ## Modification of k depending on density  Left here for documentation purposes, the first try for density, 
            ## forgotten as "RatioDens" does this way better, and dynamically.

            #if self.density < 1 && cond == "densityLess"
            #    theK = theK - k_difference
            #    n_times = zeros(theK)
            #    n_distances = zeros(theK)

            #elseif self.density < 1 && cond == "densityMore"
            #    theK = theK + k_difference
            #    n_times = zeros(theK)
            #    n_distances = zeros(theK)
            #end


            if stored_neighbours < theK
                stored_neighbours += 1
                push!(n_positions, (agent.position, agent))
                push!(modes, (agent.mode, agent.max_speed))
                n_distances[stored_neighbours] = norm(self.position - agent.position)
                push!(n_target_pos, agent.target_pos)
                n_times[stored_neighbours] = agent.timestamp
            else
                distance = norm(self.position - agent.position)
                if distance < maximum(n_distances)
                    index = argmax(n_distances)
                    n_distances[index] = distance
                    n_positions[index] = (agent.position, agent)
                    modes[index] = (agent.mode, agent.max_speed)
                    n_target_pos[index] = agent.target_pos
                    n_times[index] = agent.timestamp
                end
            end

            if stored_neighbours2 < self.k
                stored_neighbours2 += 1
                push!(modes2, agent.mode)
                n_distances2[stored_neighbours2] = norm(self.position - agent.position)
            else
                distance = norm(self.position - agent.position)
                if distance < maximum(n_distances2)
                    index = argmax(n_distances2)
                    modes2[index] = agent.mode
                    n_distances2[index] = distance
                end
            end

            tot_mod = 0
            for mode in modes2
                if mode == "explore"
                    tot_mod += 1
                end
            end

            if (tot_mod / self.k_max) < 0.65 && cond == "RatioExplo"
                if self.k-1 > 5
                    self.k -=1
                end
            elseif (tot_mod / self.k_max )> 0.65 && cond == "RatioExplo"
                if self.k+1 < self.k_max
                    self.k +=1
                end
            end

            if (self.density) < 0.35 && cond == "RatioDens"
                if self.k-1 < self.k_max
                    self.k +=1
                end
            elseif (self.density) >= 0.35 && cond == "RatioDens"
                if self.k+1 > self.k_min
                    self.k -=1
                end
            end


        end
    end

    avg_density_distance = mean(density_distances)
    self.density = (n + 1)/(pi * avg_density_distance ^ 2)

    return n_target_pos, n_times, n_positions, modes
end


function set_target!(self::Particle, n_target_pos::Array, n_times::Array, n_modes::Array, fast_speed::Float64, cond::String)

    most_recent_ts = 0
    if n_times == []
        most_recent_ts = -1
    else
        most_recent_ts = maximum(n_times)
    end

    #Neighbour's information supersedes own information   
    if most_recent_ts > self.timestamp
        # Delete own information
        self.target_pos = []
        self.timestamp = 0

        # Save neighbour information
        index = argmax(n_times)
        self.n_target_pos = self.attraction_point = n_target_pos[index]
        self.n_timestamp = self.used_timestamp = n_times[index]
        self.mode = "track"

    # If neighbour's information is old, use own information
    elseif self.target_pos != []
        self.mode = "track"
        self.attraction_point = self.target_pos
        self.used_timestamp = self.timestamp
        self.n_target_pos = []
        self.n_timestamp = 0
    else
        self.mode = "explore"
        self.attraction_point = []
        self.used_timestamp = 0
        self.n_target_pos = []
        self.n_timestamp = 0
    end

    # Making agents Flee
    if ("track", fast_speed) in n_modes && self.max_speed != fast_speed  && cond == "SlowFleeing"
        self.mode = "explore"
        self.attraction_point = []
        self.used_timestamp = 0
        self.n_target_pos = []
        self.n_timestamp = 0
    end
end

function integrate_neighbour_info(self::Particle, agent_list::Array, k_difference::Int, cond::String)
    n_target_pos, n_times, n_positions, n_modes = get_neighbour_info(self, agent_list, k_difference, cond)
    return n_target_pos, n_times, n_positions, n_modes
end

function clear_memory!(self::Particle)
    if self.target_pos != [] && self.timer > (self.timestamp + self.memory)
        self.target_pos = []
        self.timestamp = 0
    end
    if self.n_target_pos != [] && self.timer > (self.n_timestamp + self.memory)
        self.n_target_pos = []
        self.n_timestamp = 0
    end
    if self.attraction_point != [] && self.timer > (self.used_timestamp + self.memory)
        self.mode = "explore"
        self.timestamp = self.n_timestamp = self.used_timestamp = 0
        self.attraction_point = []
    end
end

function set_attraction_velocity!(self::Particle)
    if self.attraction_point != []
        self.attract_vel = (self.w * self.velocity) + (self.c * rand(1)) .*
                            (self.attraction_point - self.position)
    else
        self.attract_vel = self.w .* self.velocity
    end
end

function set_repulsion_velocity!(self::Particle, n_positions::Array, cond::String)

    self.repel_vel = [0., 0.]

    # Constant Repulsion Block
    # self.rep_radius = self.rep_radius_min

    # Adaptive Repulsion Block
    if self.mode == "explore" && self.rep_radius < self.rep_radius_max
        self.rep_radius += 0.1
    elseif self.mode == "track" && self.rep_radius > self.rep_radius_min
        self.rep_radius -= 0.75
    end

    s = pi * self.rep_radius ^ 2
    alpha_r = sqrt(s / self.k)

    for position in n_positions
        vector = self.position - position[1]
        distance = norm(vector)
        if cond == "NewRepulsion"
            self.repel_vel += ((alpha_r / distance) ^ self.d) * (vector / distance) * (position[2].density/self.density)
        else
            self.repel_vel += ((alpha_r / distance) ^ self.d) * (vector / distance)
        end
    end
end

function move_agent!(self::Particle)
    # Speed limiter
    self.velocity = self.attract_vel + self.repel_vel
    speed = norm(self.velocity)
    if speed > self.max_speed
        self.velocity = (self.max_speed / speed) * self.velocity
    end

    # Boundary conditions
    waypoint = self.position + self.velocity
    if abs(waypoint[1]) > self.boundary
        if waypoint[1] > self.boundary
            waypoint[1] = self.boundary
        else
            waypoint[1] = -self.boundary
        end
    end

    if abs(waypoint[2]) > self.boundary
        if waypoint[2] > self.boundary
            waypoint[2] = self.boundary
        else
            waypoint[2] = -self.boundary
        end
    end

    self.position = waypoint

end


####################################################################################################################################
#                                                                                                                                  #
#                                                                                                                                  #
#                                                                                                                                  #
#                                                      Target                                                                      #
#                                                                                                                                  #
#                                                                                                                                  #
####################################################################################################################################


@with_kw mutable struct Target
    # User defined settings
    name::String
    sensing_range::Float64
    max_speed::Float64
    boundary::Float64

    # Evasion settings
    d::Float64
    rep_radius::Float64
    policy::String

    # Movement Setting
    encounter::Int = 0
    encounter_limit::Int
    jump_timer::Int
    turn_limit::Float64
    timer = 0
    tracked = false
    tracked_slow = false
    tracked_fast = false

    # State Parameters
    position::Vector = [rand(Uniform(-boundary, boundary)),
                        rand(Uniform(-boundary, boundary))]
    heading::Float64 = rand(Uniform(-180, 180))
    velocity::Vector = [cosd(heading) * max_speed, sind(heading) * max_speed]
    waypoint = [rand(Uniform(-boundary, boundary)),
                rand(Uniform(-boundary, boundary))]
    evade = false
end

# Target Functions
function find_agents(self::Target, agent_list::Array, cond::String)
    agent_pos = []
    in_range = 0
    in_range_slow = 0
    in_range_fast = 0
    in_range_agent = []
    for agent in agent_list
        if norm(self.position - agent.position) <= self.sensing_range
            in_range += 1

            if agent.max_speed==max_speed
                in_range_slow += 1
            end
            if agent.max_speed==max_speed_fast
                in_range_fast += 1
            end

            push!(agent_pos, agent.position)
            push!(in_range_agent, agent)
        end
    end



    # Lowest performing agent during tracking is eliminated if there are more than 0.2 time the total of agents tracking the target.
    # Considerated as a second, closeup, communication, in this simulations case.
    if size(in_range_agent, 1) > size(agent_list, 1)*0.2 && cond == "PerfOnlyTs"|| cond == "PerfOnlyMs" || cond == "PerfOnlyNr" # 0.2 might be the best
        conditionList = []
        modified = 0

        if cond == "PerfOnlyTs"
            for agent in in_range_agent
                push!(conditionList, agent.timestamp)
            end
            modified = in_range_agent[argmin(conditionList)]

        elseif cond == "PerfOnlyMs"
            for agent in in_range_agent
                push!(conditionList, agent.max_speed)
            end
            modified = in_range_agent[argmin(conditionList)]
        else cond == "PerfOnlyNr"
            for agent in in_range_agent
                push!(conditionList, norm(self.position - agent.position))
            end
            modified = in_range_agent[argmax(conditionList)]
        end

        if modified.target_pos != [] # Clear memory without timer
            modified.target_pos = []
            modified.timestamp = 0
        end
        if modified.n_target_pos != []
            modified.n_target_pos = []
            modified.n_timestamp = 0
        end
        if modified.attraction_point != []
            modified.mode = "explore"
            modified.timestamp = modified.n_timestamp = modified.used_timestamp = 0
            modified.attraction_point = []
        end

    end




    if in_range > 0
        self.tracked = true

        if in_range_fast > 0
            self.tracked_fast = true
        else
            self.tracked_fast = false
        end
        if in_range_slow > 0
            self.tracked_slow = true
        else
            self.tracked_slow = false
        end
    else
        self.tracked = false
    end

    return agent_pos, in_range
end

function evasive_velocity!(self::Target, agent_pos::Array, in_range::Int)
    self.velocity = [0., 0.]
    if in_range > 0
        self.encounter += 1

        s = pi * self.rep_radius ^ 2
        alpha_r = sqrt(s / in_range)
        for position in agent_pos
            vector = self.position - position
            distance = norm(vector)
            self.velocity += ((alpha_r / distance) ^ self.d) *
                                (vector / distance)
        end

        self.heading = atand(self.velocity[2], self.velocity[1])
        speed = norm(self.velocity)

        if speed > self.max_speed
            self.velocity = (self.max_speed / speed) * self.velocity
        end

    else
        self.velocity = [self.max_speed * cosd(self.heading),
                        self.max_speed * sind(self.heading)]

        if self.encounter > 0
            self.encounter -= 1
        end
    end
end

function non_evasive_velocity!(self::Target)
    vec_to_wp = self.waypoint - self.position
    if norm(vec_to_wp) <= (1.5) || self.timer >= 200
        self.waypoint = [rand(Uniform(-self.boundary, self.boundary)),
                            rand(Uniform(-self.boundary, self.boundary))]
        self.timer = 0
    else
        self.timer += 1
    end

    req_heading = atand(vec_to_wp[2], vec_to_wp[1])
    heading_delta = req_heading - self.heading

    if heading_delta > 180
        heading_delta -= 360
    elseif heading_delta < -180
        heading_delta += 360
    end

    if abs(heading_delta) > self.turn_limit
        if heading_delta > 0
            self.heading += self.turn_limit
        else
            self.heading -= self.turn_limit
        end
    else
        self.heading = req_heading
    end

    self.velocity = [self.max_speed * cosd(self.heading),
                        self.max_speed * sind(self.heading)]
end

function avoid_target(self::Target, target_list::Array)
    targets_in_range = 0
    conflicts = []
    avoid = false
    for target in target_list
        if target.name == self.name
            continue
        elseif norm(target.position - self.position) <= self.sensing_range
            targets_in_range += 1
            push!(conflicts, target.position)
            avoid = true
        end
    end

    return avoid, conflicts, targets_in_range
end

function move_target!(self::Target, agent_list::Array, target_list::Array, cond::String)
    avoid, conflicts, targets_in_range = avoid_target(self, target_list)

    if avoid
        evasive_velocity!(self, conflicts, targets_in_range)
    else
        agent_pos, in_range = find_agents(self, agent_list, cond)
        if self.policy == "ne"
        non_evasive_velocity!(self)
        elseif self.policy == "e"
            evasive_velocity!(self, agent_pos, in_range)
        elseif self.policy == "mix" && self.encounter < self.encounter_limit
            evasive_velocity!(self, agent_pos, in_range)
        elseif self.policy == "mix" &&
                (self.encounter >= self.encounter_limit || self.evade == false)
            non_evasive_velocity!(self)
            self.jump_timer -= 1
            if self.jump_timer != 0
                self.evade = false
            else
                self.encounter = 0
                self.evade = true
                self.jump_timer = 30
            end
        end
    end

    self.position += self.velocity

    # Reflect off boundaries
    if abs(self.position[1]) > self.boundary
        if self.position[1] > self.boundary  # Right boundary
            overshoot = self.position[1] - self.boundary
            self.position[1] -= overshoot
            if self.heading > 0
                self.heading = 180 - self.heading
            else
                self.heading = -180 - self.heading
            end

        elseif self.position[1] < -self.boundary  # Left boundary
            overshoot = self.position[1] + self.boundary
            self.position[1] -= overshoot
            if self.heading > 0
                self.heading = 180 - self.heading
            else
                self.heading = -180 - self.heading
            end
        end
    end

    if abs(self.position[2]) > self.boundary
        if self.position[2] > self.boundary  # Top boundary
            overshoot = self.position[2] - self.boundary
            self.position[2] -= overshoot
        elseif self.position[2] < -self.boundary  # Bottom boundary
            overshoot = self.position[2] + self.boundary
            self.position[2] -= overshoot
        end
        self.heading = -self.heading
    end
end



####################################################################################################################################
#                                                                                                                                  #
#                                                                                                                                  #
#                                                                                                                                  #
#                                                      More edits                                                                  #
#                                                                                                                                  #
#                                                                                                                                  #
####################################################################################################################################


# Multithreading function
function parallel_function_integrate(agent_list::Array, k_difference::Int, cond::String)
    agent_pass = Vector{Tuple}(undef, length(agent_list)) 

    Threads.@threads for i in eachindex(agent_list)
        agent = agent_list[i]
        n_target_pos, n_times, n_positions, n_modes = integrate_neighbour_info(agent, agent_list, k_difference, cond)
        agent_pass[i] = (agent, n_target_pos, n_times, n_positions, n_modes)
    end

    return agent_pass
end



# Static edit for k for given conditions
function edit_k_function(agent_list::Array, cond::String, k_difference::Int)
    # "lessTracking", "moreTracking", "lessExploring", "moreExploring"
    for agent in agent_list

        if cond == "lessTracking"
            if agent.mode == "track"
                agent.k = agent.oldk - k_difference
            else
                agent.k = agent.oldk
            end
        end

        if cond == "moreTracking"
            if agent.mode == "track"
                agent.k = agent.oldk + k_difference
            else
                agent.k = agent.oldk
            end
        end

        if cond == "lessExploring"
            if agent.mode == "explore"
                agent.k = agent.oldk - k_difference
            else
                agent.k = agent.oldk
            end
        end

        if cond == "moreExploring"
            if agent.mode == "explore"
                agent.k = agent.oldk + k_difference
            else
                agent.k = agent.oldk
            end
        end  

        if cond == "incitateSpeed"
            if norm(agent.velocity) < agent.max_speed/8 # Velocity is much lower than max_speed / x (4) so we try to make the agent move
                agent.k = agent.oldk - k_difference
                #agent.k = agent.oldk - (agent.oldk-1) # Just go away from the closer one
            else
                agent.k = agent.oldk
            end
        end

    end

end


# Fleeing high density spaces, does not work in this state, as closing in on the tracked target might trigger this function and setoff fleeing mode.
# Still given for documentation purposes.
function set_fleeing!(self::Particle, n_positions::Array)

    if self.mode == "track"
        return 0
    end

    distances = []
    fleeing = []
    agents = []
    density = []
    for agent in n_positions
        norme = norm(self.position - agent[1])
        if length(distances) < 6
            push!(distances, norme)
            push!(fleeing, agent[2].fleeing)
            push!(agents, agent[1])
            push!(density, agent[2].density)
        elseif norme < maximum(distances)
            index = argmax(distances)
            distances[index] = norme
            agents[index] = agent[1]
            fleeing[index] = agent[2].fleeing
            density[index] = agent[2].density
        end
    end

    highdens = argmax(density)


    if self.density > (40/((self.boundary*2).^2)*2)
        if sum(fleeing) == 0
            self.fleeing = 1

            newpos = agents[highdens]
            self.attraction_point = self.position - newpos*10
        end

    end
    
end


function attraction_of_low_density(self::Particle, agent_list::Array, n_particles::Int, ) # Not used as not practical, and considered as a less good idea, but kept here just in case.
    lowest_dens = 5 # If self is low (very low) look in all it's neighbours to find if one is in high density and can come (set a new attraction point to make him come closer)
    lowest_agent = self
    for agent in agent_list
        if lowest_dens > agent.density
            lowest_dens = agent.density
            lowest_agent = agent
        end
    end

    if agent.density < (n_particles/((self.boundary*2).^2)/2) # Density lower than rohs / 2
        self.attraction_point = lowest_agent.position * 50 # Agent can't go out of boundary anyway
    end

end


# Not plugged in the code as it raises problems of the multiplication of potential loops of communication, defeating part of the purpose of the communication loop
# Used to calculate the density before the rest of the communications, it is used to edit k sooner, and might not be the best way to go, our precedent strategies use
# The k of one iteration before, and produce great results.
function get_neighbour_densities(self::Particle, agent_list::Array, k_difference::Int, cond::String, n_particles::Int)
    density_distances = []
    n_forgets = []

    n = 6   # Number of nearest neighbours to consider when calculating local density

    for agent in agent_list
        if agent.name == self.name
            continue
        
        else 
            distance = norm(self.position - agent.position)

            # This block to gather distances between n neighbours to calculate local density
            if length(density_distances) < n
                push!(density_distances, distance)
                push!(n_forgets, agent.forget_k)
            elseif distance < maximum(density_distances)
                density_distances[argmax(density_distances)] = distance
                n_forgets[argmax(n_forgets)] = agent.forget_k
            end
        end
    end

    avg_density_distance = mean(density_distances)
    self.density = (n + 1)/(pi * avg_density_distance ^ 2)

    # Modification of k depending on density
    if self.density < 1 && cond == "densityLess"
        theK = theK - k_difference
        n_times = zeros(theK)
        n_distances = zeros(theK)

    elseif self.density < 1 && cond == "densityMore"
        theK = theK + k_difference
        n_times = zeros(theK)
        n_distances = zeros(theK)
    # If local density > rhos*2, i.e there are too much agents here
    # Another way to go about the fleeingHighDensity problematic, not better than the precedent one, also given for documentation purposes
    elseif self.density > (n_particles/((self.boundary*2).^2)*2) && cond == "fleeHighDensity" && self.mode != "track" 
        if sum(n_forgets) == 0
            self.oldk = self.k
            self.k = 0
            self.forget_k = 1
        end
        n_times = zeros(theK)
        n_distances = zeros(theK)
    end

    if self.forget_k > 0 # Followup flee high density
        self.forget_k += 1
        if self.forget_k > 2
            self.forget_k = 0
            self.k = oldk
        end
    end
    return n_times, n_distances
end



####################################################################################################################################
#                                                                                                                                  #
#                                                                                                                                  #
#                                                                                                                                  #
#                                                      Main                                                                        #
#                                                                                                                                  #
#                                                                                                                                  #
####################################################################################################################################



# Main function  
function main(all_values)

    if all_values === 0
        return 0
    end

    all_score = []
    all_proportions = []
    all_loc_density = []

    all_score_slow = []
    all_score_fast = []

    iter_max, w, c, max_speed, max_speed_fast, n_particles, n_neighbours,
    width, boundaries, memory, n_fs, sensing_radius, rep_radius_max, rep_radius_min,
    d_def, n_targets, target_speed, detection_radius, turn_limit, movement_policy,
    encounter_limit, jump_timer, rep_radius,score_file, proportion_file, density_file, k_difference, cond = all_values

    ##saving settings Uncomment if you dont put the names in main
    #if n_fs != 0
    #    type = "hetero"
    #else
    #    type = "homo"
    #end

    #file_name = type * "_" * string(n_particles) * "_" * string(n_fs) * "_k" * string(n_neighbours) * "_mem" * string(memory) * "_rep" * string(rep_radius_min) * "max" * string(rep_radius_max) * "_" * movement_policy

    #score_file = file_name * "_score.csv"
    ## explore_file = file_name & "_explore.csv"
    #proportion_file = file_name * "_engagement.csv"

    for boundary in boundaries    # for different environment sizes

    # boundary = boundaries
    #for nf in [0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,42,44,46,48,50,52,54,56,58,60]   # for different environment sizes

    # The boundary loop can be modified as above to generate data depending on the quantity of fast agents for a set boundary size.
    # Do not forget to only put one value for "boundaries", though it might be possible to do two loops, i would advise against it as it would take much time
    # Better to execute for multiple boundaries in the main file, in order to stop the execution more easily

        k = n_neighbours[1]

        if size(boundary) != 1 # In case you use a zipped table to set k for each boundary size
            boundary = boundary[1]
            k = boundary[2]
        end

        k = n_neighbours[1]
        nf = n_fs[1]
        time_limit = memory[1]
        println("Start k", k, " Memory ", time_limit, " Fast Agents ", nf)
        println("Width: ", boundary*2)
        Random.seed!(5512)

        score = 0
        score_slow = 0
        score_fast = 0

        pos_history::Vector{Array{Array{String,1},1}} = []
        target_history::Vector{Array{Array{Float64,1},1}} = []
        exploit_proportion = []
        average_loc_density = []

        agent_list = []
        target_list = []

        for i in range(1, stop=n_particles-nf)
            name = i
            agent = Particle(name=name, k=k, oldk=k, max_speed=max_speed, boundary=boundary,
                            memory=time_limit, sensing_range=sensing_radius, w=w,
                            c=c, rep_radius_max=rep_radius_max,
                            rep_radius_min=rep_radius_min, d=d_def)
            agent.boundary -= sensing_radius/2
            push!(agent_list, agent)
        end


        if nf != 0
            for i in range(n_particles-nf+1, stop=n_particles)
                name = i
                agent = Particle(name=name, k=k, oldk=k, max_speed=max_speed_fast, boundary=boundary,
                                memory=time_limit, sensing_range=sensing_radius, w=w,
                                c=c, rep_radius_max=rep_radius_max,
                                rep_radius_min=rep_radius_min, d=d_def)
                agent.boundary -= sensing_radius/2
                push!(agent_list, agent)
            end
        end


        for i in range(1, stop=n_targets)
            name = "Target " * string(i)
            target = Target(name=name, sensing_range=detection_radius,
                            max_speed=target_speed, boundary=boundary, d=d_def,
                            rep_radius=rep_radius, policy=movement_policy,
                            encounter_limit=encounter_limit, jump_timer=jump_timer,
                            turn_limit=turn_limit)

            if target.policy == "mix" || target.policy == "e"
                target.evade = true
            else
                target.evade = false
            end
            push!(target_list, target)
        end

        iteration = 0

        while iteration < iter_max

            write_pos_current = []
            pos_current = []
            target_current = []
            iter_exploit_proportion = 0
            loc_density = 0

            for target in target_list
                move_target!(target, agent_list, target_list, cond)
                push!(target_current, target.position)
                if target.tracked
                    score += 1
    
                    if target.tracked_fast
                        score_fast += 1
                    end
                    if target.tracked_slow
                        score_slow += 1
                    end
                end
    
            end

            push!(target_history, target_current)

            for agent in agent_list
                # This saves tt, sf, and ss onto the position file so that the video processing file knows if an agent is tracking or not
                # Also differentiates between frast agents and slow agents
                if agent.mode == "track"
                    save_pos = ["tt" * string(agent.position[1]), string(agent.position[2])]
                    iter_exploit_proportion += 1
                elseif agent.max_speed == max_speed_fast
                    save_pos = ["sf" * string(agent.position[1]), string(agent.position[2])]
                else
                    save_pos = ["ss" * string(agent.position[1]), string(agent.position[2])]
                end
                update_target_info!(agent, target_list)
                clear_memory!(agent)
                push!(pos_current, [agent.position])
                push!(write_pos_current, save_pos)
            end

            push!(pos_history, write_pos_current)

            push!(exploit_proportion, iter_exploit_proportion/n_particles)

            # "lessTracking", "moreTracking", "lessExploring", "moreExploring", "SlowFleeing"
            edit_k_function(agent_list, cond, k_difference)

            agent_pass = parallel_function_integrate(agent_list, k_difference, cond)

            for agent_tuple in agent_pass
                agent, n_target_pos, n_times, n_positions, n_modes = agent_tuple
                set_target!(agent, n_target_pos, n_times, n_modes, max_speed_fast, cond)

                if cond == "FleeHighDens" # Flee High Density modification
                    set_fleeing!(agent, n_positions)
                end

                set_attraction_velocity!(agent)
                set_repulsion_velocity!(agent, n_positions, ContinuousUnivariateDistribution)
                loc_density += agent.density
            end

            push!(average_loc_density, loc_density/n_particles)

            for agent in agent_list
                move_agent!(agent)
                agent.timer += 1
            end

            iteration += 1
            # println(iteration)
        end
        score /= (iter_max * n_targets)
        score_slow /= (iter_max * n_targets)
        score_fast /= (iter_max * n_targets)

        println("k = ", k, " fast agents: ", nf, " score: ", score,  " score slow agents: ", score_slow, " score fast agents: ", score_fast)
        # println("Iter Exploit Proportion: ", mean(exploit_proportion))
        push!(all_score, score)
        push!(all_score_slow, score_slow)
        push!(all_score_fast, score_fast)

        push!(all_proportions, mean(exploit_proportion))
        push!(all_loc_density, mean(average_loc_density))

        file_name1 = "julia_test_target.csv"
        file_name2 = "julia_test_agents.csv"

        #CSV.write(file_name1, DataFrame(target_history), writeheader=false)
        #CSV.write(file_name2, DataFrame(pos_history), writeheader=false)
    end

    #print(all_score)
    #print(all_loc_density)

    CSV.write(score_file, DataFrame([all_score, all_score_slow, all_score_fast], :auto), writeheader=false)
    CSV.write(proportion_file, DataFrame([all_proportions], :auto), writeheader=false)
    CSV.write(density_file, DataFrame([all_loc_density], :auto), writeheader=false)
    #CSV.write(network_file, DataFrame([all_clustering], :auto), writeheader=false)

    println("Done")
end