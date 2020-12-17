/*******************************************************************************************
	example.c
	This code illustrates an example of Rate Monotonic Real Time OS with 
	service sequencing in Linux.
	
	This is achieved by running POSIX threads with the SCHED_FIFO command that
	allows the user to occupy a processor with a thread (or task in Kernel-
	space) until the thread is run to completion. Nomrally the Linux CFS would
	be sending interrupts to prevent the thread from hogging the proccessor. 
	By using SCHED_FIFO we can achieve a soft RT system in Linux userspace. In
	addition we are already provided with preemption included in POSIX by ass-
	igning each thread with a priority number. 
	Being able to use userspace Linux comes with some benefits:
	
		1. Fast engineering: Linux is a well known system among developers
		   and writing C in userspace is fast and easy.
		2. Safety: The separation between the kernelspace and the userspace
		   makes the system safe from outside influence.
	
	There are, however, downsides aswell. We can only achieve timers in the
	millisecond range - or even tens of milliseconds. Kernelspace can achieve
	timers in the microsecond range and hard RTOS systems can be even faster.
	Another downside is that our process might experience some drift over time.
	Thus a soft RTOS with Linux is not applicable to all RT-systems but for
	those systems where some fraction of milliseconds of drift over an interval
	is not detrimental - it is a good and low-cost choice. For larger projects,
	kernel modules could be written and integrated into the system. However, in
	most cases this is more work than simply using a hard RTOS system.
		* Use kernel patches for RT semaphore support.
		* Use FUTEX instead of standard POSIX sempahores.

	Summary: 
		The three criteria for RTOS that we have fulfilled are:
		1. Run to completion (aka core affinity).
		2. Preemption.
		3. Accurate (minimal drift) timers and time stamps.
	This all adds up to a >>PREDICTABLE RESPONSE<<.
	
	BUILD INFO:
	This code was build on a Raspberry Pi 3+ with 4 cores.
		Core 0: Mostly Linux kernel but also does load balancing.
		Core 1: Sequencer.
		Core 2: Even threads.
		Core 3: Odd threads.
	
	100 Hz sequencer:
	[Gives semaphores to all other services]
	S1: 50 Hz: every 2nd sequencer loop.
	S2: 20 Hz: every 5th 
	S3: 10 Hz: every 10th
	S4:  5 Hz: every 20th
	S5:  2 Hz: every 50th
	S6:  1 Hz: every 100th
	S7:  1 Hz: every 100th

	According to Rate Monotonic (RM) theory this leads to priorities:
	Sequencer = RT_MAX   @ 100 Hz
	       S1 = RT_MAX-1 @  50 Hz
	       S2 = RT_MAX-2 @  20 Hz
	       etc.

	2020-12-10
	Johannes Westman
	**************************************************************************
	HOW TO:
		- Make sure that you are running the program on a multicore system
		  in Linux. use "lscpu" in the Linuxterminal to see how many cores
		  you have and which cores are active.
		- Use command "Make build" to create the file.
		- Run the file with 'sudo' privileges.
	TO DO:  
		- eliminate printf calls and make systemlog calls or in-memory
		  logger.

********************************************************************************************/

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h> //POSIX

#include<pthread.h>
#include<sched.h>
#include<time.h>
#include<semaphore.h>

#include<syslog.h>
#include<sys/time.h>
#include<sys/sysinfo.h>
#include<errno.h>

#define USEC_PER_MSEC (1000)
#define NANOSEC_PER_MSEC (1000000)
#define NANOSEC_PER_SEC (1000000000)
#define NUM_CPU_CORES (4)
#define TRUE (1)
#define FALSE (0)
#define NUM_THREADS (8)

// TYPES OF CLOCKS IN USERSPACE:
//	CLOCK_REALTIME
//	CLOCK_MONOTONIC
//	CLOCK_MONOTONIC_RAW : generally most precise.
//	CLOCK_REALTIME_COARSE
//	CLOCK_MONOTONIC_COARSE

#define MY_CLOCK_TYPE CLOCK_MONOTONIC_RAW

/*
In general the clock type will depend on the hardware and the application.
Monotonic means time since some event excluding suspended time.
Raw means ignoring NTP synchronizations - closer to hardware time.
*/

int abortTest = FALSE;
int abortS1=FALSE, abortS2=FALSE, abortS2=FALSE, abortS3=FALSE, abortS4=FALSE,
    abortS5=FALSE, abortS6=FALSE, abortS7=FALSE;
sem_t semS1, semS2, semS3, semS4, semS5, semS6, semS7;
struct timespec start_time_val;
double start_realtime;

typedef struct{
  int threadIdx;
  unsigned long long sequencePeriods;
} threadParams_t

//initialize threads
void* Sequencer(void* threadp);
void* Service_1(void* threadp);
void* Service_2(void* threadp);
void* Service_3(void* threadp);
void* Service_4(void* threadp);
void* Service_5(void* threadp);
void* Service_6(void* threadp);
void* Service_7(void* threadp);
void* threadFunctions[8] = {Sequencer, Service_1, Service_2, Service_3,
			    Service_4, Service_5, Service_6, Service_7};
double getTimeMsec(void);
double realtime(struct timespec *tsptr);
void print_scheduler(void);




void main(void) {
  struct timespec current_time_val, current_time_res;
  double current_realtime, current_realtime_res;
  int i, rc, scope;
  cpu_set_t threadcpu; //read the CPU_SET manual in Linux.
  pthread_t threads[NUM_THREADS];
  threadParams_t threadParams[NUM_THREADS];  //parameters for threads
  pthread_attr_t rt_checd_attr[NUM_THREADS]; //attributes for threads
  int rt_max_prio, rt_min_prio, cpuidx;      //get the prioriy numbers and the number of cores
  struct sched_param rt_param[NUM_THREADS];  //parameters for the RTOS threads
  struct sched_param main_param;	     //parameters for running threads
  pthread_attr_t main_attr;		     
  pid_t mainpid;
  cpu_set_t allcpuset;

  printf("Starting Demonstration\n") //remove this for faster runtime
  clock_gettime(MY_CLOCK_TYPE, &start_time_val); //from time.h
  start_realtime = realtime(&start_time_val);
  clock_gettime(MY_CLOCK_TYPE, &current_time_val);
  current_realtime = realtime(&current_time_val);
  clock_getres(MY_CLOCK_TYPE, &current_time_res);
  current_realtime_res = realtime(&current_time_res);
  printf("START High Rate Sequencer @ sec = %6.9lf with resultion %6.9lf\n"
	 , (current_realtime - start_realtime), current_realtime_res);
  printf("System has %d processors configured and %d available. \n", 
	  get_nprocs_conf(), get_nprocs());
  
  CPU_ZERO(&allcpuset); //Linux methods to set affinity. 
		        //CPU_ZERO clears the set.

  for(i=0; i<NUM_CPU_CORES; i++) {
    CPU_SET(i, &allcpuset);
  }
  printf("Using CPUs %d from total available\n", CPU_COUNT(&allcpuset));

  //initialize all semaphores
  if (sem_init(&semS1, 0, 0)) {
    printf("Failed to initialize semaphore S1\n");
    exit(-1); 
  }
  if (sem_init(&semS2, 0, 0)) {
    printf("Failed to initialize semaphore S2\n");
    exit(-1); 
  }
  if (sem_init(&semS3, 0, 0)) {
    printf("Failed to initialize semaphore S3\n");
    exit(-1); 
  }
  if (sem_init(&semS4, 0, 0)) {
    printf("Failed to initialize semaphore S4\n");
    exit(-1); 
  }
  if (sem_init(&semS5, 0, 0)) {
    printf("Failed to initialize semaphore S5\n");
    exit(-1); 
  }
  if (sem_init(&semS6, 0, 0)) {
    printf("Failed to initialize semaphore S6\n");
    exit(-1); 
  }
  if (sem_init(&semS7, 0, 0)) {
    printf("Failed to initialize semaphore S7\n");
    exit(-1); 
  }

  mainpid = getpid(); //LINUX returns the processID of the calling process.

  //get the priority levels for the system.
  rt_max_prio = sched_get_priority_max(SCHED_FIFO);
  rt_min_prio = sched_get_priority_min(SCHED_FIFO);
  printf("RT max prio: %d\nRT min prio: %d\n", rt_max_prio, rt_min_prio);
  
  //get the scheduling parameters for mainpid.
  rc = sched_getparam(mainpid, &main_param);
  main_param.sched_priority = rt_max_prio;
  rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
  if(rc < 0) {
    perror("main_param");
  }
  print_scheduler(); //prints out info regarding process
  
  //gets the contention scope of the system. 
  pthread_attr_getscope(&main_attr, &scope);
  if(scope == PTHREAD_SCOPE_SYSTEM){
    printf("PTHREAD_SCOPE_SYSTEM\n");
    //the thread competes with all other threads on all processes on the system.
  }
  else if(scope == PTHREAD_SCOPE_PROCESS){
    printf("PTHREAD_SCOPE_PROCESS\n");
    //competes with all other threads in the same process.
  }
  else{
    printf("PTHREAD SCOPE UNKNOWN");
  }

  //distribute threads on 2 cores
  for(i=0; i<NUM_THREADS; i++){
    CPU_ZERO(&threadcpu);
    //run even threads on core 2
    if(i%2 == 0) {
      cpuidx=2;
    }
    //run odd threads on core 3
    else {
      cpuidx=3; 
    }
      CPU_SET(cpuidx, &threadcpu);

  //initialize pthread.
  rc = pthread_attr_init(&rt_sched_attr[i]);
  //set thread to inherit scheduling from attributes instead of parent thread.
  rc = pthread_attr_setinheritsched(&rt+sched+attr[i], PTHREAD_EXPLICIT_SCHED);
  //sets scheduling to Real Time appropriate protocil (FIFO).
  rc = pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO);
  // affinity determines on which CPU's the thread is allowed to run.
  // Here the thread ID=rt_sched_attr[i] is set to run on threadcpu. 
  rc = pthread_attr_setaffinity_np(&rt_sched_attr[i], sizoeof(cpu_set_t), &threadcpu);
  // sets the schedule priority for thread i. Note the order!
  rt_param[i].sched_priority = rt_max_prio-i;
  // sets the scheduling parameters for thread i
  pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]);
  threadParams[i].threadIdx=i;
  }

  printf("Service threads will run on %d CPU cores\n", CPU_COUNT(&threadcpu));

  //Creating the service threads, note the order.
  //Service_i = RT_MAX-i @ 50, 20, 10, 5, 2, 1 Hz
  for(i=1; i<NUM_THREADS; i++){
    rt_param[i].sched_priority = rt_max_prio-i; //set thread priority
    pthread_attr_setschedparam(&rt_sched_attr[i], &rt_param[i]); //set parameters
	
    rc = pthread_create(&threads[i],		//pointer to thread descriptor
			&rt_sched_attr[i],	//use specific attributes for this thread
			threadFunctions[i],	//select the service.
    			(void*)&(threadParams[1]) //address of parameters
    			);
    if(rc<0){
      perror("pthread_create for Service %d\n", i);
    }
    else{
      printf("pthread_create successful for Service %d\n", i);
    }
  }
  
  //Creating the Sequencer
  printf("Start Sequencer\n");
  threadParams[0].sequencePeriods=2000;
  
  //run n core 1
  CPU_ZERO(&threadcpu);
  cpuidx = 1;
  CPU_SET(cpuidx,&threadcpu);
  rc = pthread_setaffinity_np(&rt_sched_attr[0], sizeof(cpu_set_t), &threadcpu);
  
  // Sequencer: RT_MAX	@ 100 Hz
  rt_param[0].sched_priority = rt_max_prio;
  pthread_attr_setschedparam(&rt_sched_attr[0], &rt_param[0]);
  rc = pthread_create(&threads[0], &rt_sched_attr[0], threadFunctions[0],
			(void*)&(threadParams[0]));
  if(rc<0){
    perror("pthread_create for Sequencer\n")
  }
  else{
    printf("pthread_create successful for Sequencer\n");
  }
  
  //join threads
  for(i=0; i<NUM_THREADS; i++){
    pthread_join(threads[i], NULL);
  }
  printf("TEST COMPLETE\n");
}


void* Sequencer(void* threadp){
  struct timespec current_time_val;
  struct timespec delay_time = {0,10000000};
  struct timespec remaining_time;
  double current_realtime;
  int rc;
  unsigned long long seqCnt = 0;
  threadParams_t *threadParams = (threadParams_t *)threadp;
  
  //clock settings
  clock_gettime(MY_CLOCK_TYPE, &current_time_val);
  current_realtime =  realtime(&current_time_val);
  syslog(LOG_CRIT, "Sequencer Thread @ sec = %6.9lf\n", current_realtime);
  //ToDo: probably not LOG_CRIT??

  //this loop releases the services as per their frequency.
  //written in 'list form' for clarity.
  do{
      //put the thread to sleep for interval
      if(rc = clock_nanosleep(CLOCK_MONOTONIC, 0, &delay_time, &remaining_time) != 0){
        perror("Sequencer nanosleep");
        exit(-1);
      }

    //Service_1 = RT_MAX - 1 	@ 50 Hz
    if((seqCnt%2) == 0) sem_post(&semS1);

    //Service_2 = RT_MAX - 2 	@ 20 Hz
    if((seqCnt%5) == 0) sem_post(&semS2);

    //Service_3 = RT_MAX - 3 	@ 10 Hz
    if((seqCnt%10) == 0) sem_post(&semS3);

    //Service_4 = RT_MAX - 4 	@ 5 Hz
    if((seqCnt%20) == 0) sem_post(&semS4);

    //Service_5 = RT_MAX - 5 	@ 2 Hz
    if((seqCnt%50) == 0) sem_post(&semS5);

    //Service_6 = RT_MAX - 6 	@ 1 Hz
    if((seqCnt%100) == 0) sem_post(&semS6);

    //Service_7 = RT_MAX - 6 	@ 1 Hz
    if((seqCnt%100) == 0) sem_post(&semS7);
    
    //increment counter
    seqCnt++;

  }while( !abortTest && (seqCnt < threadParams->sequencePeriods) );

  //increment semaphores
  sem_post(&semS1); sem_post(&semS2); sem_post(&semS3);
  sem_post(&semS4);sem_post(&semS5);
  sem_post(&semS6); sem_post(&semS7); sem_post(&semS8);
  
  //abort threads
  abortS1=TRUE; abortS2=TRUE; 
  abortS3=TRUE; abortS4=TRUE; 
  abortS5=TRUE; abortS6=TRUE; 
  abortS7=TRUE;
  
  pthread_exit((void*)0);
}

/*
	These next functions simulate RT-services. They don't really
	do anything significant, they just simulate RT applications
	in Linux. All services do the same thing.
*/

void* Service_1(void* threadp){ 
  struct timespec current_time_val;
  unsigned long long S1Cnt = 0; //Service counter
  threadParams_t *threadParams = (threadParams_t*)threadp;

  clock_gettime(MY_CLOCK,&current_time_val);
  current_realtime = realtime(&current_time_val);
  syslog(LOG_CRIT, "S1 thread @ sec = %6.9lf\n", (current_realtime - start_realtime) );

  while( !abortS1 ){
    sem_wait(&semS1); //lock semaphore
    S1Cnt++;	      //increment counter
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    syslog(LOG_CRIT, "S1 50Hz on core %d for release %llu @ sec=%6.9lf\n", 
	   sched_getcpu(), S1Cnt, (current_realtime - start_realtime) );
  }
  
  pthread_exit((void*)0); //exit thread
}


void* Service_2(void* threadp){ 
  struct timespec current_time_val;
  unsigned long long S2Cnt = 0;
  threadParams_t *threadParams = (threadParams_t*)threadp;

  clock_gettime(MY_CLOCK,&current_time_val);
  current_realtime = realtime(&current_time_val);
  syslog(LOG_CRIT, "S2 thread @ sec = %6.9lf\n", (current_realtime - start_realtime) );

  while( !abortS2 ){
    sem_wait(&semS2);
    S2Cnt++;
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    syslog(LOG_CRIT, "S2 20Hz on core %d for release %llu @ sec=%6.9lf\n", 
	   sched_getcpu(), S2Cnt, (current_realtime - start_realtime) );
  }
  
  pthread_exit((void*)0); //exit thread
}


void* Service_3(void* threadp){ 
  struct timespec current_time_val;
  unsigned long long S3Cnt = 0;
  threadParams_t *threadParams = (threadParams_t*)threadp;

  clock_gettime(MY_CLOCK,&current_time_val);
  current_realtime = realtime(&current_time_val);
  syslog(LOG_CRIT, "S3 thread @ sec = %6.9lf\n", (current_realtime - start_realtime) );

  while( !abortS3 ){
    sem_wait(&semS3);
    S3Cnt++;
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    syslog(LOG_CRIT, "S3 20Hz on core %d for release %llu @ sec=%6.9lf\n", 
	   sched_getcpu(), S3Cnt, (current_realtime - start_realtime) );
  }
  
  pthread_exit((void*)0); //exit thread
}


void* Service_4(void* threadp){ 
  struct timespec current_time_val;
  unsigned long long S4Cnt = 0;
  threadParams_t *threadParams = (threadParams_t*)threadp;

  clock_gettime(MY_CLOCK,&current_time_val);
  current_realtime = realtime(&current_time_val);
  syslog(LOG_CRIT, "S4 thread @ sec = %6.9lf\n", (current_realtime - start_realtime) );

  while( !abortS4 ){
    sem_wait(&semS4);
    S4Cnt++;
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    syslog(LOG_CRIT, "S2 20Hz on core %d for release %llu @ sec=%6.9lf\n", 
	   sched_getcpu(), S4Cnt, (current_realtime - start_realtime) );
  }
  
  pthread_exit((void*)0); //exit thread
}

void* Service_5(void* threadp){ 
  struct timespec current_time_val;
  unsigned long long S5Cnt = 0;
  threadParams_t *threadParams = (threadParams_t*)threadp;

  clock_gettime(MY_CLOCK,&current_time_val);
  current_realtime = realtime(&current_time_val);
  syslog(LOG_CRIT, "S5 thread @ sec = %6.9lf\n", (current_realtime - start_realtime) );

  while( !abortS5 ){
    sem_wait(&semS5);
    S5Cnt++;
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    syslog(LOG_CRIT, "S5 20Hz on core %d for release %llu @ sec=%6.9lf\n", 
	   sched_getcpu(), S5Cnt, (current_realtime - start_realtime) );
  }
  
  pthread_exit((void*)0); //exit thread
}


void* Service_6(void* threadp){ 
  struct timespec current_time_val;
  unsigned long long S6Cnt = 0;
  threadParams_t *threadParams = (threadParams_t*)threadp;

  clock_gettime(MY_CLOCK,&current_time_val);
  current_realtime = realtime(&current_time_val);
  syslog(LOG_CRIT, "S6 thread @ sec = %6.9lf\n", (current_realtime - start_realtime) );

  while( !abortS6 ){
    sem_wait(&semS6);
    S6Cnt++;
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    syslog(LOG_CRIT, "S6 20Hz on core %d for release %llu @ sec=%6.9lf\n", 
	   sched_getcpu(), S6Cnt, (current_realtime - start_realtime) );
  }
  
  pthread_exit((void*)0); //exit thread
}


void* Service_7(void* threadp){ 
  struct timespec current_time_val;
  unsigned long long S7Cnt = 0;
  threadParams_t *threadParams = (threadParams_t*)threadp;

  clock_gettime(MY_CLOCK,&current_time_val);
  current_realtime = realtime(&current_time_val);
  syslog(LOG_CRIT, "S7 thread @ sec = %6.9lf\n", (current_realtime - start_realtime) );

  while( !abortS7 ){
    sem_wait(&semS7);
    S7Cnt++;
    clock_gettime(MY_CLOCK_TYPE, &current_time_val);
    current_realtime = realtime(&current_time_val);
    syslog(LOG_CRIT, "S7 20Hz on core %d for release %llu @ sec=%6.9lf\n", 
	   sched_getcpu(), S7Cnt, (current_realtime - start_realtime) );
  }
  
  pthread_exit((void*)0); //exit thread
}


//converts time to milliseconds
double getTimeMsec(void){
  struct timespec event_ts = {0,0};
  clock_gettime(MY_CLOCK_TYPE, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}


//prints the type of scheduler that is active.
//for RTOS we need SCHED_FIFO, in other cases, exit program.
void print_scheduler(void){
  int schedType;
  schedType = sched_getscheduler(getpid());
  switch(schedType){
    case SCHED_FIFO:
    	printf("Pthread policy is SCHED_FIFO\n");
	break;
    case SCHED_OTHER:
    	printf("Pthread policy is SCHED_OTHER\n");
	exit(-1);
	break;
    case SCHED_RR:
	printf("Pthread policy is SCHED_RR\n");
	exit(-1);
	break;
    default:
	printf("Pthread policy is UNKNOWN\n");
	exit(-1);
  }
}
