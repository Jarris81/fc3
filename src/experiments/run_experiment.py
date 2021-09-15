import time
import pandas as pd
from optparse import OptionParser
import actions
import planners
from execution_models import RLGS, SimpleSystem, RLDSClone
import interferences
import util.setup_env as setup_env
import os

choices_model = ["simple", "rlgs", "rlds_clone"]
choices_scenario = ["tower", "hand_over",  "stick"]

"""
Build a tower with the provided plan. 
- During execution, a block is placed to its original position, which should show 
interference in the real world. 
- we want to show that we 
"""


experiment_configs = {
        "tower": {
            "scene": setup_env.setup_tower_env(),
            "planner": planners.TowerPlanner(),
            "actions": [actions.GrabBlock(), actions.PlaceOn(), actions.PlaceSide()],
            "interferences": interferences.get_tower_interferences()
        },
        "hand_over": {
            "scene": setup_env.setup_hand_over_env(),
            "planner": planners.HandOverPlanner(),
            "actions": [actions.GrabBlock(), actions.PlaceGoal(), actions.HandOver()],
            "interferences": interferences.get_hand_over_interferences(),
        },
        "stick": {
            "scene": setup_env.setup_stick_pull_env(),
            "planner": planners.StickPullPlanner(),
            "actions": [actions.GrabBlock(), actions.PlaceGoal(), actions.GrabStick(),
                        actions.PullBlockToGoal(), actions.PlaceStick()],
            "interferences": interferences.get_stick_interferences(),
        }
    }


def run_experiment(model_name, scenario, interference_num, use_real_robot, tracking,
                   verbose=False, record=True):

    scenario_config = experiment_configs[scenario]
    C, scene_objects = scenario_config["scene"]
    action_list = scenario_config["actions"]
    planner = scenario_config["planner"]
    interference = scenario_config["interferences"][interference_num]

    # Choose model
    if model_name == "rlgs":
        exec_model_func = RLGS
    elif model_name == "simple":
        exec_model_func = SimpleSystem
    elif model_name == "rlds_clone":
        exec_model_func = RLDSClone
    else:
        print(f"Model {model_name} is not implemented!")
        return

    exec_model = exec_model_func(C, verbose=verbose, use_real_robot=use_real_robot, use_tracking=tracking)
    C.view()

    init_complete, init_time = exec_model.init_system(action_list, planner, scene_objects)
    exec_time = 0

    if not init_complete:
        print("Plan is not feasible!")
        C.view_close()
    else:
        # setup the task
        exec_model.setup()
        exec_time = exec_model.run(interference)

    goal_full = exec_model.is_goal_fulfilled()
    if goal_full:
        print("Plan finished!")
    else:
        print("Plan not finished!")

    if use_real_robot:
        exec_model.move_home()
    C.view_close()

    if record:
        experiment_results = {
            "exec_time": exec_time,
            'setup_time': init_time,
            "goal_reached": str(goal_full),
            "scenario": scenario,
            'model_name': model_name,
            'interference_num': interference_num,
            'interference_desc': interference.description,
            'date': time.strftime("%Y%m%d%H%M"),
            'real_robot': str(use_real_robot),

        }
        log_experiment_row(experiment_results)


def log_experiment_row(data):
    df = pd.DataFrame(data, index=[0])

    def assure_path_exists(path):
        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            os.makedirs(directory)

    path = f"../../data/experiments.csv"
    assure_path_exists(path)
    is_new_file = os.path.exists(path)
    df.to_csv(path, mode='a', header=not is_new_file, index=False)


def run_all_experiments(count=1, use_real_robot=False):
    for model in choices_model:
        for scenario in choices_scenario:
            for interference in range(len(experiment_configs[scenario]["interferences"])):
                for i in range(count):
                    # run_experiment(model, scenario, interference_num=inter, record=True, use_real_robot=use_real_robot, tracking=False)
                    os.system(f"python {os.getcwd()}/run_experiment.py --{model=} --{scenario=} --{interference=}")

if __name__ == '__main__':
    parser = OptionParser()

    parser.add_option("-m", "--model", dest="model_name", default="rlgs",
                      help="Which system to use", type="choice", choices=choices_model)

    parser.add_option("-v", "--verbose", dest="verbose", default=False, action="store_true",
                      help="don't print status messages to stdout")

    parser.add_option("-s", "--scenario", dest="experiment_name",
                      help="Define the experiment", type="choice", choices=choices_scenario)

    parser.add_option("-i", "--interference", dest="interference_num", default=0,
                      help="Define interference", type="int")

    parser.add_option("-r", "--use_real_robot", dest="use_real_robot", default=False, action="store_true",
                      help="use real robots or simulate")

    parser.add_option("-t", "--tracking", dest="tracking", default=False, action="store_true",
                      help="use tracking with OptiTrack")

    parser.add_option("-a", "--all", dest="run_all", default=False, action="store_true",
                      help="run all experiments")

    (options, args) = parser.parse_args()

    if options.run_all:
        run_all_experiments(count=1,
                            use_real_robot=options.use_real_robot)

    else:
        run_experiment(model_name=options.model_name,
                       scenario=options.experiment_name,
                       interference_num=options.interference_num,
                       use_real_robot=options.use_real_robot,
                       tracking=options.tracking and options.use_real_robot,
                       verbose=options.verbose)
