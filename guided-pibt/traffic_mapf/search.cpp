


#include "search.hpp"

#ifdef USE_BPR_HEURISTIC
#include "TrajLNS.h"
#include "bpr.hpp"
#endif


namespace TrafficMAPF{
std::chrono::nanoseconds t;
//a astar minimized the opposide traffic flow with existing traffic flow

#ifdef FOCAL_SEARCH
void update_focal(pqueue_min_f& open, pqueue_min_jam& focal, int& f_min, int& f_bound){
    if(open.empty())
        return;
    if (focal.empty()){
        f_min = open.top()->get_f();
        f_bound = f_min * FOCAL_SEARCH;
    }

    while(!open.empty() && open.top()->get_f() <= f_bound){
        focal.push(open.top());
        open.pop();
    }
    return;
}
#endif
s_node singleShortestPath(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht,std::vector<int>& traffic, Traj& traj,
    MemoryPool& mem, int start, int goal)
{
    traj.clear();
    traj.push_back(start);
    int neighbors[4];

    while(traj.back() != goal){
        int curr = traj.back();
        getNeighborLocs(env,neighbors,curr);
        int min_h = INT_MAX;
        int next = -1;
        for(int i=0;i<4;i++){
            if(neighbors[i] == -1)
                continue;
            assert(!ht.empty());
            int h = get_heuristic(ht,env, traffic, flow, neighbors[i]);
            if(h < min_h){
                min_h = h;
                next = neighbors[i];
            }
        }
        traj.push_back(next);
    }
    return s_node(goal,traj.size(),0,0,traj.size());
}


s_node aStarOF(SharedEnvironment* env,
#ifdef USE_BPR_HEURISTIC
    const TrajLNS& lns,  // BPR parameter
#endif
    std::vector<Int4>& flow,
    HeuristicTable& ht,std::vector<int>& traffic, Traj& traj,
    MemoryPool& mem, int start, int goal)
{
    // DEBUG: Track when search is called
    static int search_call_count = 0;
    search_call_count++;
    if (search_call_count <= 5) {
        std::cerr << "DEBUG: aStarOF called #" << search_call_count
                  << " from " << start << " to " << goal << std::endl;
    }

    // TimePoint start_time = std::chrono::steady_clock::now();
    mem.reset();
    // t+=std::chrono::steady_clock::now()-start_time;
    // cout<<"reset time:"<<t.count()<<endl;


    int expanded=0;
    int generated=0;
    int h;

    //  s_node* root = mem.generate_node(start,0,manhattanDistance(start,goal,env),0,0);
    if(ht.empty())
#ifdef USE_BPR_HEURISTIC
        h = manhattanDistance(start,goal,env) * TrajLNS::COST_SCALE;  // Scale heuristic
#else
        h = manhattanDistance(start,goal,env);  // Baseline: no scaling
#endif
    else
#ifdef USE_BPR_HEURISTIC
        h = get_heuristic(ht,env, traffic, flow, start);  // Already scaled by get_heuristic()
#else
        h = get_heuristic(ht,env, traffic, flow, start);  // Baseline: no scaling
#endif
    

    
    s_node* root = mem.generate_node(start,0, h,0,0,0);

    if (start == goal){
        traj.clear();
        traj.push_back(start);
        return *root;
    }

    #ifdef FOCAL_SEARCH
    int f_min = h;
    int f_bound = f_min * FOCAL_SEARCH;
    pqueue_min_f open;
    pqueue_min_jam focal;
    re_f ref;
    re_jam rej;

    #else
    pqueue_min_of open;
    re_of re;
    #endif

    open.push(root);



    

    int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff;
    int next_d1, next_d2, next_d1_loc, next_d2_loc;
    int temp_op, temp_vertex;
    double tie_breaker;

    s_node* goal_node = nullptr;
    int neighbors[4];
    int next_neighbors[4];



#ifdef FOCAL_SEARCH
    while (open.size() + focal.size()  > 0){
        update_focal(open,focal,f_min,f_bound);
        s_node* curr = focal.pop();

#else
    while (open.size() > 0){

        s_node* curr = open.pop();

#endif

        curr->close();

        if (curr->id == goal){
            goal_node = curr;
            break;
        }
        expanded++;

        // DEBUG: Detect infinite loops
        if (expanded % 100000 == 0) {
            std::cerr << "DEBUG: Search #" << search_call_count
                      << " expanded " << expanded << " nodes, curr->id=" << curr->id
                      << ", goal=" << goal << std::endl;
        }
        if (expanded > 10000000) {
            std::cerr << "ERROR: Search #" << search_call_count
                      << " expanded > 10M nodes! Aborting." << std::endl;
            break;
        }


        // std::cout<<curr->id<<":"<< curr->get_f()<<","<< curr->get_h() <<","<< curr->get_op_flow()<<"," << curr->get_all_vertex_flow()<<"," << std::endl;
        getNeighborLocs(env,neighbors,curr->id);

        
        for (int i=0; i<4; i++){
            int next = neighbors[i];
            if (next == -1){
                continue;
            }

#ifdef USE_BPR_HEURISTIC
            // BPR cost: already includes full edge cost (free-flow + congestion penalty)
            int edge_cost = get_bpr_edge_cost(lns, curr->id, next);
            // DEBUG: Print suspicious edge costs
            static int debug_count = 0;
            if (debug_count < 20 && edge_cost >= 10000) {
                std::cerr << "DEBUG: High edge cost " << edge_cost << " from " << curr->id << " to " << next << std::endl;
                debug_count++;
            }
            cost = curr->g + edge_cost;  // No extra +1 or +COST_SCALE
#else
            // Baseline: traditional cost +1
            cost = curr->g + 1;
#endif
            tie_breaker = curr->tie_breaker;


            if (traffic[next] != -1 ){
                int candidates[4] = { next + 1,next + env->cols, next - 1, next - env->cols};
                if (curr->id  == candidates[traffic[next]])
					continue;            
            }

            assert(next >= 0 && next < env->map.size());
            depth = curr->depth + 1;

            //moving direction
            //flow
            op_flow = curr->op_flow; //op_flow is contra flow
            all_vertex_flow = curr->all_vertex_flow;

            if(ht.empty())
#ifdef USE_BPR_HEURISTIC
                h = manhattanDistance(next,goal,env) * TrajLNS::COST_SCALE;  // Scale to match g (1000x)
#else
                h = manhattanDistance(next,goal,env);  // Baseline: no scaling
#endif
            else
#ifdef USE_BPR_HEURISTIC
                h = get_heuristic(ht,env, traffic, flow, next);  // Already scaled by get_heuristic()
#else
                h = get_heuristic(ht,env, traffic, flow, next);  // Baseline: no scaling
#endif

            diff = next - curr->id;
            d = get_d(diff,env);


            temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));

            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = 1;
            for (int j=0; j<4; j++){
                temp_vertex += flow[next].d[j];                
            }

            if (OBJECTIVE == OBJ::O_VC){
                op_flow += temp_op;
            }



#ifdef FOCAL_SEARCH
            if (OBJECTIVE == OBJ::O_VC || OBJECTIVE == OBJ::VC ){
                all_vertex_flow+= (temp_vertex-1) /2;
            }

            if (OBJECTIVE == OBJ::SUM_OVC){
                all_vertex_flow  += (temp_vertex-1) /2 + temp_op;
            }            
#else
            if (OBJECTIVE == OBJ::O_VC || OBJECTIVE == OBJ::VC){
                cost+= (temp_vertex-1) /2;
            }

            if (OBJECTIVE == OBJ::SUM_OVC){
                cost = cost + (temp_vertex-1) /2 + temp_op;
            }
#endif

            if(OBJECTIVE == OBJ::SUI_TG){
                tie_breaker = (0.5 * (double)temp_vertex/(double)env->num_of_agents + 0.5 * (double)flow[next].d[(d+2)%4]/(double)env->num_of_agents);
            }

            if(OBJECTIVE == OBJ::SUI_TC){
                tie_breaker += (0.5 * (double)temp_vertex/(double)env->num_of_agents + 0.5 * (double)flow[next].d[(d+2)%4]/(double)env->num_of_agents)/(double)env->max_h;
            }

            p_diff = 0;
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
            }



            s_node temp_node(next,cost,h,op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow,  all_vertex_flow);

            if (!mem.has_node(next)){
                s_node* next_node = mem.generate_node(next,cost,h,op_flow, depth,all_vertex_flow);
                next_node->parent = curr;
                next_node->tie_breaker = tie_breaker;
#ifdef FOCAL_SEARCH
                if (next_node->get_f() <= f_bound)
                    focal.push(next_node);
                else
                    open.push(next_node);

#else
                open.push(next_node);
#endif
                generated++;
            }
            else{ 
                s_node* existing = mem.get_node(next);

                if (!existing->is_closed()){

#ifdef FOCAL_SEARCH
                    if (existing->get_f() <= f_bound){
                        //in focal list
                        if (rej(temp_node,*existing)){
                            existing->g = cost;
                            existing->parent = curr;
                            existing->depth = depth;
                            existing->tie_breaker = tie_breaker;
                            existing->set_all_flow(op_flow,  all_vertex_flow);
                            focal.decrease_key(existing);
                        }
                    }
                    else{
                        //not in focal list
                        if (ref(temp_node,*existing)){
                            existing->g = cost;
                            existing->parent = curr;
                            existing->depth = depth;
                            existing->tie_breaker = tie_breaker;
                            existing->set_all_flow(op_flow,  all_vertex_flow);
                            open.decrease_key(existing);
                            
                        }
                    }

#else
                    if (re(temp_node,*existing)){
                        existing->g = cost;
                        existing->parent = curr;
                        existing->depth = depth;
                        existing->tie_breaker = tie_breaker;
                        existing->set_all_flow(op_flow,  all_vertex_flow);
                        open.decrease_key(existing);
                    }
#endif
                }
                else{
                    // closed , check if re expansion needed
#ifdef FOCAL_SEARCH
                    // if (ref(temp_node,*existing)){
                    //     existing->g = cost;
                    //     existing->parent = curr;
                    //     existing->depth = depth;
                    //     existing->set_all_flow(op_flow,  all_vertex_flow);
                    //     if (existing->get_f() <= f_bound){
                    //         focal.push(existing);
                    //     }
                    //     else{
                    //         open.push(existing);
                    //     }
                    // }
#else

                if (re(temp_node,*existing)){ 
                    std::cout << "error in aStarOF: re-expansion" << std::endl;
                    assert(false);
                    exit(1);
                }
#endif

                } 
            }
        }
            

          
    }

    // std::cout << "expanded: " << expanded << std::endl;
    // std::cout << "generated: " << generated << std::endl;
    if (goal_node == nullptr){
        std::cout << "error in aStarOF: no path found "<< start<<","<<goal << std::endl;
        assert(false);
        exit(1);
    }

    traj.resize(goal_node->depth);
    s_node* curr = goal_node;
    for (int i=goal_node->depth-1; i>=0; i--){
        traj[i] = curr->id;
        curr = curr->parent;
    }
    // std::cout<< goal_node->get_f()<<","<< goal_node->get_h() <<","<< goal_node->get_op_flow()<<"," << goal_node->get_all_vertex_flow()<<"," << std::endl;
    // std::cout<< root->get_f()<<","<< root->get_h()<<","<< root->get_op_flow()<<"," << root->get_all_vertex_flow()<<"," << std::endl;
    // exit(1);
    return *goal_node;
}
}

