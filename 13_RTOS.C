// RTOS Framework - Spring 2018
// J Losh

// Student Name:
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// xx_rtos.c   Single-file with your project code
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs

#define PUSH_BUTTON1  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))//PF4
#define PUSH_BUTTON2  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) //PA7
#define PUSH_BUTTON3  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) //PA6
#define PUSH_BUTTON4  (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))//PA5
#define PUSH_BUTTON5  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))//PB4

#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))//PF2
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) //PA2
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4)))//PD2
#define YELLOW_LED (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))//PD1
#define ORANGE_LED (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))//PA4

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
void *system_sp;
struct semaphore
{
  uint16_t count;
  uint16_t queueSize;
  uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_RUN        2
#define STATE_READY      3 // has run, can resume at any time
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore
#define STATE_DELAYED    5 // has run, but now awaiting timer
#define Max_Char 25
#define MAX_TASKS 10       // maximum number of valid tasks

uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint8_t previousPriority=0;



char str[Max_Char];
uint8_t position[10]={0};
uint8_t fields=0;
char type[10]={'\0'};
uint32_t pid_Cmd=0;
uint8_t keycount;

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t skipCount;             // skip count of priority
  uint8_t priority;              // 0=highest, 7=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint32_t ticks;
  uint32_t T1;
  uint32_t T2;
  uint32_t totalTime;
  uint32_t cpuTime;
  char name[16];                 // name of task used in ps command
  void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

void rtosInit()
{
  uint8_t i;
  // no tasks running
  taskCount = 0;
  // clear out tcb records
  for (i = 0; i < MAX_TASKS; i++)
  {
    tcb[i].state = STATE_INVALID;
    tcb[i].pid = 0;
  }
  // REQUIRED: initialize systick for 1ms system timer
//   NVIC_ST_CTRL_R  = 0X00;
   NVIC_ST_RELOAD_R  = 0X9C3F;
   NVIC_ST_CURRENT_R  = 0x00;
   NVIC_ST_CTRL_R  = 0X07;
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    yield();

    return UART0_DR_R & 0xFF;
}

char* getString(uint8_t field)
{
    return &str[position[field]];
}

uint32_t key_getPID(uint8_t field)
{
   uint32_t i1;
   char* b[10];
   b[0]=&str[position[field]];
   i1=atoi(b[0]);
   return i1;
}

void check_key_press()
{
    uint8_t i,j,f,alpha,num;
    alpha=0;
    num=0;
    f=0;
    keycount=0;
            while(keycount<Max_Char)
                {
                  char c =toupper(getcUart0());
                  keycount++;
                  i=keycount-1;
                    if(keycount>0)
                    {
                        if(c==8)
                        {
                            keycount--;
                            keycount--;
                        }
                    }
                   if(c==13)
                        {
                       str[i]='\0';
                       keycount--;
                       break;
                        }

                   if(c!=8)
                       str[i]=c;

               }
            putsUart0(str);
            for(j=0;j<keycount;j++)
               {

                 if((str[j]>=65 && str[j]<=90) || (str[j]>=97 && str[j]<=122)) // A-Z : 65-90 a-z:97-122 numbers 48-57
                     {
                     alpha++;
                     if(alpha==1)
                       {
                         position[f]=j;
                         type[f]='a';
                         f++;
                       }
                     }

                 else if(str[j]>=48 && str[j]<=57)
                     {
                     num++;
                     if(num==1)
                     {
                         position[f]=j;
                         type[f]='n';
                         f++;
                     }
                     }

                 else
                 {
                     str[j]='\0';
                     alpha=0;
                     num=0;
                 }
               }
               fields = f-1;
}

/* For state we can use switch case
 *  switch (tcb[taskCurrent].state)
        {
        case 1:
        putsUart0("STATE_INVALID");
        case 2:
        putsUart0("STATE_UNRUN");
        case 3:
        putsUart0("STATE_RUN");
        case 4:
        putsUart0("STATE_READY");
        case 5:
        putsUart0("STATE_DELAYED");
        }
 */

void CommandFunction()
{
   char* a;
   uint8_t i=0;
   uint8_t j=0;
   char b[25]={"\0"};
   struct semaphore *temp;
   temp=tcb[taskCurrent].semaphore;
// key_getPID()
    if(strcmp(&str[position[0]],"PS")==0 && fields==0)
     {
        putsUart0("\n\rIt's a PS command");
        putsUart0("\n\rPID\t\tNAME\t\tCPU USAGE\t\tSEMAPHORE");
        sprintf(b,"\n\r%dh",tcb[taskCurrent].pid);
        putsUart0(b);
        putsUart0("\t\t");
        putsUart0(tcb[taskCurrent].name);
        putsUart0("\t\t");
        putsUart0("\t\t");
       if(temp==keyPressed)
            putsUart0("keyPressed");
        else if(temp==keyReleased)
            putsUart0("keyReleased");
        else if(temp==flashReq)
            putsUart0("flashReq");
        else if(temp==resource)
            putsUart0("resource");
        else
            putsUart0("NA");



     }

    else if(strcmp(&str[position[0]],"IPCS")==0 && fields==0)
        {
        putsUart0("\n\rIt's a IPCS command");
        putsUart0("\n\rSemaphore : ");
        if(temp==keyPressed)
            putsUart0("keyPressed");
        else if(temp==keyReleased)
            putsUart0("keyReleased");
        else if(temp==flashReq)
            putsUart0("flashReq");
        else if(temp==resource)
            putsUart0("resource");
         else
            putsUart0("NA");
       // temp=tcb[taskCurrent].semaphore;
        putsUart0("\n\rCount : ");
        sprintf(b,"%d",temp->count);
        putsUart0(b);
        putsUart0("\n\rQueue Size : ");
        sprintf(b,"%d",temp->queueSize);
        putsUart0(b);

        }

    else if(strcmp(&str[position[0]],"KILL")==0 && fields==1)
     {
        pid_Cmd=key_getPID(1);
        putsUart0("\n\rIt's a KILL command");
        destroyThread(pid_Cmd);
     }
    else if(strcmp(&str[position[0]],"REBOOT")==0 && fields==1)
         {
        putsUart0("\n\rIt's a REBOOT command");
        __asm("    .global _c_int00\n"
                  "    b.w     _c_int00");
         }
    else if(strcmp(&str[position[0]],"PIDOF")==0 && fields==2)
            {
        a= getString(1);
        putsUart0("\n\rIt's a PID_OF command");
        while(i<taskCount)
        {
            for(j=0;j<strlen(tcb[i].name);j++)
            b[j]=toupper(tcb[i].name[j]);
            if(strcmp(a,b)==0)
            {
                sprintf(str,"\n\r%dh",tcb[i].pid);
                putsUart0("\n\rPID : ");
                putsUart0(str);
                break;
            }
            else
                i++;
        }
            }
    else
        putsUart0("\n\rThe entered command is not valid");


}

uint32_t getSP()
{
  //  __asm(" ADD SP,#8 ");
    __asm(" MOV R0,SP ");
    __asm(" BX LR ");

}
void setSP(void *stackP)
{
    //move it by 8 because SUB.W sp sp 8 is happening on a call
    __asm(" ADD SP,#8 ");
    __asm(" MOV SP,R0 ");
    __asm(" BX LR ");
}
void rtosStart()
{
  // REQUIRED: add code to call the first task to be run
  _fn fn;
  system_sp=getSP();
  taskCurrent = rtosScheduler();
  setSP(tcb[taskCurrent].sp);
  tcb[taskCurrent].state= STATE_UNRUN;
  fn=tcb[taskCurrent].pid;
  (*fn)();
  // Add code to initialize the SP with tcb[task_current].sp;

}

bool createThread(_fn fn, char name[], int priority)
{
  bool ok = false;
  uint8_t i = 0;
  bool found = false;
  // REQUIRED: store the thread name
 // add task if room in task list
       if (taskCount < MAX_TASKS)
  {
    // make sure fn not already in list (prevent reentrancy)
    while (!found && (i < MAX_TASKS))
    {
      found = (tcb[i++].pid ==  fn);
    }
    if (!found)
    {
      // find first available tcb record
      i = 0;
      while (tcb[i].state != STATE_INVALID) {i++;}
      tcb[i].state = STATE_UNRUN;
      tcb[i].pid = fn;
      tcb[i].sp = &stack[i][255];
      tcb[i].priority = priority;
      tcb[i].currentPriority = priority;
      tcb[i].skipCount = priority;
      strcpy(tcb[i].name,name);
      // increment task count
      taskCount++;
      ok = true;
    }
  }
  // REQUIRED: allow tasks switches again
  return ok;
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{// kill the tick too by checking the semaphore entry in tcb
    uint32_t i;
    uint8_t j;
    struct semaphore *s;
    i=0;
    j=0;
    for(i=0;i<taskCount;i++)
    {
        if(tcb[i].pid == fn)
        {
            tcb[i].state = STATE_INVALID;
            tcb[i].cpuTime=0;
            s=tcb[i].semaphore;
            tcb[i].semaphore=0;

            while(j>0 && j<s->queueSize)
           {
                if(i==s->processQueue[j-1])
                {
                    s->processQueue[j-1]=s->processQueue[j];
                    s->queueSize--;
                }
                j++;
           }

        }

    }
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

struct semaphore* createSemaphore(uint8_t count)
{
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {
    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
  }
  return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    tcb[taskCurrent].state=STATE_READY;
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
  // push registers, call scheduler, pop registers, return to new function

}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    // push registers, set state to delayed, store timeout, call scheduler, pop registers,
    // return to new function (separate unrun or ready processing)
    tcb[taskCurrent].ticks= tick;
    tcb[taskCurrent].state= STATE_DELAYED;
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    uint8_t i=0;
 /* for PI
  * If a Higher priority thread calls wait & if it's state is blocked,
  * then the current thread using semaphore is raised to priority of higher priority thread
  * */
    tcb[taskCurrent].semaphore = pSemaphore;
    if(pSemaphore->count>0)
    {
       pSemaphore->count--;
       if(tcb[taskCurrent].state == STATE_UNRUN )
           tcb[taskCurrent].state = STATE_READY;
    }
    else
    {
        // //Using the current task index to store into semaphore queue

        pSemaphore->processQueue[pSemaphore->queueSize]=taskCurrent;
        tcb[taskCurrent].state = STATE_BLOCKED;
        pSemaphore->queueSize++;

        //taskCurrent is the higher priority task with e.g. priority = 3 and i look for if priority more than 3 i.e. 7

        while(i<=pSemaphore->queueSize)
        {
            // so if 3<7 and both have same semaphore using assign the priority
            if((i!=taskCurrent) && (tcb[i].state!=STATE_BLOCKED) && (tcb[i].semaphore == tcb[taskCurrent].semaphore) && (tcb[taskCurrent].priority < tcb[i].priority))
            {
                previousPriority=tcb[i].priority;
                tcb[i].skipCount = tcb[taskCurrent].priority;
                tcb[i].currentPriority = tcb[taskCurrent].priority;
                //tcb[taskCurrent].state=STATE_READY;
                pSemaphore->processQueue[pSemaphore->queueSize]=i;
            }
            i++;
        }

        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }


}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    uint8_t i;
    uint16_t j;
    j=1;
    pSemaphore->count++;
    tcb[taskCurrent].state = STATE_READY;
    while(pSemaphore->queueSize>0)
    {
        //First In First Out processing for the semaphore
        for(i=0;i<MAX_TASKS;i++)
        {
            if(pSemaphore->processQueue[0]==i)
                { while(pSemaphore->count>0)
                {
                tcb[i].state=STATE_READY;// set the blocked state to ready
               tcb[i].currentPriority=previousPriority;
                tcb[i].skipCount=previousPriority;
                pSemaphore->count--;
             pSemaphore->processQueue[pSemaphore->queueSize]=0;
               while(j>0 && j<pSemaphore->queueSize)
               {
                   pSemaphore->processQueue[j-1]=pSemaphore->processQueue[j];
                   j++;
               }
                }
                }

      //  break;
        }
        pSemaphore->queueSize--;
        }
     NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: Implement prioritization to 8 levels

/*
 * lower the number higher the priority  0 higher 7 lower
 * 1st time through all run
 * treat priority number as the no of times maximum you will skip the task before it's schedule
 * skipping depends on priority number
 * if skipcount is equal to priority, don't skip it
 * Initialize skipcount to priority number
 * For priority inheritance current priority is used
 * greater than equal should be used for dynamic changes in priority : for PI
 * use idle,idle1,idle2 use diff priorities for check
 */
int rtosScheduler()
{
  bool ok;
  static uint8_t task = 0xFF;
  ok = false;
  while (!ok)
  {
    task++;
    if (task >= MAX_TASKS)
      task = 0;
    // run the task if ready or unrun and Skipc > = priority
    if((tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN) && (tcb[task].skipCount >= tcb[task].priority))
    {
        tcb[task].skipCount = 0;
        ok = true;
    }
    else
        (tcb[task].skipCount)++;
  }
  return task;
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{// for now only for co-operative
    uint8_t current_task;
   // current_task=0;

        for(current_task=0;current_task<MAX_TASKS;current_task++)
            {
            if(tcb[current_task].state==STATE_DELAYED)
            {
                tcb[current_task].ticks--;
                if(tcb[current_task].ticks==0)
                    tcb[current_task].state=STATE_READY;
            }
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{//clear pendsv bit
  //  _fn fn;
//push R4-R11

   __asm(" ADD SP,#8 ");
   __asm(" PUSH {R4-R11} ");
   NVIC_INT_CTRL_R |= NVIC_INT_CTRL_UNPEND_SV;
   tcb[taskCurrent].sp=getSP();
  taskCurrent = rtosScheduler();
   setSP(tcb[taskCurrent].sp);
   if(tcb[taskCurrent].state==STATE_READY)
   {
    __asm(" POP {R4-R11} ");
   }
   if(tcb[taskCurrent].state==STATE_UNRUN)
   {
//push lowest register number at lowest address
// so r3 to r0
       __asm( " MOV R0,#0 " );
      __asm( " PUSH {R0} " );
       __asm( " MOV R0, #0x4100 " );
              __asm( " MOV R0,R0,LSL #16 " );
              __asm( " ADD R0, #0x0000 " );
              __asm( " PUSH {R0} " );
              getPID();
              __asm( " PUSH {R0} " );//pc push
              getPIDLR();
              //LR at the end should have value of FFFFFFF( (8 dummy entries) for FFFFFFE9 (17 dummy entries)
              __asm( " SUB R0,#1 " );
              __asm( " PUSH {R0} " );//lr push
              __asm( " PUSH {R12} " );
              __asm( " PUSH {R3} " );
              __asm( " PUSH {R2} " );
              __asm( " PUSH {R1} " );
              __asm( " PUSH {R0} " );
}

__asm( " MOV LR, #0xFFFF " );
__asm( " MOV LR,LR,LSL #16 " );
__asm( " MOV R0, #0xFFF9 " );
__asm( " ADD LR,R0, " );
__asm( " BX LR " );
}

uint32_t getPID()
{
    return ((uint32_t)tcb[taskCurrent].pid | 0x00000001);
}
uint32_t getPIDLR()
{
    return ((uint32_t)tcb[taskCurrent].pid);
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO PORT F, A and D  peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOD ;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R |= 0x04;  // make bit 1 an outputs and 4 as input for pushbutton1
    GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x14;  // enable LED and PUSHBUTTON1
    GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button


    GPIO_PORTA_DIR_R |= 0x14;  // make bit 3,2 an outputs and 5,6,7 as input for pushbutton 2,3,4
    GPIO_PORTA_DR2R_R |= 0x14; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= 0xF4;  // enable LED
    GPIO_PORTA_PUR_R = 0xE0;  // enable internal pull-up for push button

    GPIO_PORTD_DIR_R |= 0x86;  // make bit 1 an outputs
    GPIO_PORTD_DR2R_R |= 0x86; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTD_DEN_R |= 0x86;  // enable LED

    GPIO_PORTB_DIR_R |= 0x00;  // make bit 4 an input
    GPIO_PORTB_DR2R_R |= 0x00; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= 0x10;  // enable LED
    GPIO_PORTB_PUR_R = 0x10;  // enable internal pull-up for push button

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;       // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

  // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
  //           5 pushbuttons, and uart
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             NOP");
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             NOP");
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    uint8_t  a=0;
    if(PUSH_BUTTON1 == 0)
        a=1;
    if(PUSH_BUTTON2 == 0)
        a=2;
    if(PUSH_BUTTON3 == 0)
        a=4;
    if(PUSH_BUTTON4 == 0)
        a=8;
    if(PUSH_BUTTON5 == 0)
        a=16;
    return a;
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(10000);
    ORANGE_LED = 0;
    yield();
  }
}
void idle2()
{
  while(true)
  {
    YELLOW_LED = 1;
    waitMicrosecond(10000);
    YELLOW_LED = 0;
    yield();
  }
}


void idle3()
{
  while(true)
  {
    RED_LED = 1;
    waitMicrosecond(10000);
    RED_LED = 0;
    yield();
  }
}


void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}

void oneshot()
{
  while(true)
  {
    wait(flashReq);
    YELLOW_LED = 1;
    sleep(100);
    YELLOW_LED = 0;
    post(flashReq);
  }

}

void oneshot1()
{
  while(true)
  {
    wait(flashReq);
    RED_LED = 1;
    sleep(1000);
    RED_LED = 0;
    check_key_press();
    CommandFunction();
    //post(flashReq);
  }
}
void oneshot2()
{
  while(true)
  {
    wait(flashReq);
    GREEN_LED = 1;
    sleep(1000);
    GREEN_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
    BLUE_LED=1;
    waitMicrosecond(1000);
    BLUE_LED=0;
  // give another process a chance to run
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
    post(resource);
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createThread(flash4Hz, "Flash4Hz", 0);
    }
    if ((buttons & 8) != 0)
    {
      destroyThread(flash4Hz);
    }
    if ((buttons & 16) != 0)
    {
      setThreadPriority(lengthyFn, 4);
    }
    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}

void important()
{
    while(true)
    {
      wait(resource);
      BLUE_LED = 1;
      sleep(1000);
      BLUE_LED = 0;
      post(resource);
    }
}

void shell()
{
  while (true)
  {
     putsUart0("\n\r");
     check_key_press();
     CommandFunction();
    // REQUIRED: add processing for the shell commands through the UART here
  }
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
  bool ok;


  // Initialize hardware
  initHw();
  rtosInit();

  putsUart0("\n\rUART0 Initialized");
  putsUart0("\n\r6314 Project");

  // Power-up flash
  GREEN_LED = 1;
  waitMicrosecond(250000);
  GREEN_LED = 0;
  waitMicrosecond(250000);

  // Initialize semaphores
  keyPressed = createSemaphore(1);
  keyReleased = createSemaphore(0);
  flashReq = createSemaphore(5);
  resource = createSemaphore(1);

  // Add required idle process
ok =  createThread(idle, "Idle", 7);
// Add other processes

ok &= createThread(lengthyFn, "LengthyFn", 6);
//ok &= createThread(flash4Hz, "Flash4Hz", 2);
ok &= createThread(oneshot, "OneShot", 2);
ok &= createThread(readKeys, "ReadKeys", 6);
ok &= createThread(debounce, "Debounce", 6);
ok &= createThread(important, "Important", 0);
//ok &= createThread(uncooperative, "Uncoop", 5);
ok &= createThread(shell, "Shell", 4);

  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;
}
