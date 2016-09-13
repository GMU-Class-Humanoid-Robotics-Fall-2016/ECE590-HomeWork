import time
import signal

start_time = 0
new_start_time = 0
delta_time = 0

def main():
   global start_time
   global new_start_time
   global delta_time
   print("Demonstration of blocking real time for 10 seconds")
   blocking_real_time = True
   start_time = time.time()
   new_start_time = start_time
   # delta time is the change in time to complete the loop
   while blocking_real_time:
      delta_time = time.time() - new_start_time
      new_start_time = time.time()
      print(delta_time)
      # sleeping frees the cpu for other processes
      time.sleep(0.2 - ((time.time()-new_start_time)%0.2))
      # end the loop after 10 seconds
      if((time.time()-start_time)>10): 
         blocking_real_time = False
   
   print("Demonstration of non-blocking real time for 10 seconds")
   non_blocking_real_time = True
   start_time = time.time()
   new_start_time = start_time
   while non_blocking_real_time:
       delta_time = time.time() - new_start_time
       new_start_time = time.time()
       # busy waiting
       while delta_time < 0.2:
          delta_time = time.time() - new_start_time
       print(delta_time)
       if((time.time()-start_time)>10):
          non_blocking_real_time = False

   print("Demonstration of interrupt based real time for 10 seconds")
   interrupt_based_real_time = True
   start_time = time.time()
   new_start_time = time.time()
   # Interrupt after 0.2 seconds
   signal.signal(signal.SIGALRM,handler)
   signal.setitimer(signal.ITIMER_REAL,0.2,0.2)
   while interrupt_based_real_time:
      if((time.time() - start_time)<10):
         pass
      else:
         signal.setitimer(signal.ITIMER_REAL,0)
         interrupt_based_real_time = False
    
   print("Demonstration of simulation time real time for 10 seconds")
   simulation_time_real_time = True
   start_time = time.time()
   new_start_time = time.time()
   new_sim_start_time = time.time()
   while simulation_time_real_time:
      delta_time = time.time() - new_start_time
      sim_delta_time = time.time() - new_sim_start_time
      new_start_time = time.time()
      new_sim_start_time = time.time()
      print(sim_delta_time, delta_time)
      time.sleep(0.2 - ((time.time()-new_start_time)%0.2))
      if((time.time() - start_time) > 10):
        simulation_time_real_time = False


def handler(signum, frame):
   global delta_time
   global new_start_time
   delta_time = time.time() - new_start_time
   new_start_time = time.time()
   print(delta_time)


if __name__=='__main__':
   main()
