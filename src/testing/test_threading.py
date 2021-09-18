import libry as ry
import util.setup_env as setup_env
from threading import Thread
import multiprocessing as mp
import time



if __name__ == "__main__":

    C, scene_objects = setup_env.setup_stick_pull_env()

    is_done = False


    def do_computation(CCC, values, q):

        CCC.view()

        result = all([bool(x % 2) for x in values])
        time.sleep(1)
        q.put(result)
        return result

    def get_feasible_chain():

        while not is_done:

            valos = [
                [9, 3, 5, 1],
                [8, 2, 3, 4],
            ]
            # with mp.Pool(processes=2) as pool:
            #     results = [pool.apply_async(do_computation, (C, values)) for values in valos]
            #     print([res.get(timeout=1) for res in results])

            queues = [mp.Queue() for i in range(2)]

            all_proceses = []

            for valo, q in zip(valos, queues):
                process1 = mp.Process(target=do_computation, args=(C,valo, q))
                process1.start()
                all_proceses.append(process1)

            for process in all_proceses:
                process.join(1)

            for q in queues:
                print(q.get(1))



    thread1 = Thread(target=get_feasible_chain)
    t_start = time.time()
    thread1.start()

    b1 = C.getFrame("b1")

    for i in range(100):
        pos = b1.getPosition()
        pos[2] += 0.1
        b1.setPosition(pos)
        time.sleep(0.5)



    is_done = True

    thread1.join(10)




    print("should end thread now")


    print(f"ended in {time.time() - t_start}")