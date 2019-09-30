// RTOS - Fall 2018, Framework by Professor J Losh
// Project done by: Student Name: Sujjatul Islam

// final version: 10/08/2018 corrected delete thread function

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
#include <stdlib.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))  // off-board red LED on PA5
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))  // off-board orange LED on PA7
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))  // off-board yellow LED on PA6
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4)))  // off-board green LED on PB4
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))  // on-board blue LED on PF2

#define PB0         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))   // off board push button 0 on PA2
#define PB1         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))   // off board push button 1 on PA3
#define PB2         (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))   // off board push button 2 on PA4
#define PB3         (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))   // off board push button 3 on PB6
#define PB4         (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 7*4)))   // off board push button 4 on PB7

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

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
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

struct _tcb
{
  uint8_t state;                 // see STATE_ values above
  void *pid;                     // used to uniquely identify thread
  void *sp;                      // location of stack pointer for thread
  uint8_t priority;              // 0=highest, 15=lowest
  uint8_t currentPriority;       // used for priority inheritance
  uint8_t skipCount;             // used for implementing skip priority
  uint32_t ticks;                // ticks until sleep complete
  uint64_t runtime;              // used for calculating CPU Percentage
  uint64_t runtimeIir;           // IIR filtered runtime
  char name[16];                 // name of task used in ps command
  void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];  // 1024 byte stack for each thread

struct threadTable               // directory for keeping thread information
{
  void *pid;
  uint8_t priority;
  char name[16];
} thread[MAX_TASKS];

// priority, preemption and priority inheritance on by default
bool priority = true;
bool preemptive = true;
bool pi = true;

// shell command
#define MAX_CHARS 80
#define MAX_FIELDS 3
char strInput[MAX_CHARS+1];
char fieldType[MAX_FIELDS];
uint8_t fieldCount;               // Number of fields input by user
uint8_t fieldPosition[MAX_FIELDS];

// Other Variables
struct semaphore *tempSemaphore;  // used for wait and post function
void *tempThread;                 // used for destroyThread function
uint8_t taskSemaphore;            // task holding semaphore
uint32_t *systemSp;               // system stack pointer
uint64_t totalTime = 0;           // used for calculating CPU Percentage
uint64_t timePeriod = 0;          // total time period (IIR filtered)

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// setup directory for created thread
void createThreadTable(uint8_t i)
{
    thread[i].pid = tcb[i].pid;
    thread[i].priority = tcb[i].priority;
    strCopy(thread[i].name, tcb[i].name);
}

//get current stack pointer address
uint32_t *getSp()
{
    __asm(" MOV R0, SP");
    __asm(" BX LR");
}

//set stack pointer to passed argument
void setSp(void *address)
{
    __asm(" MOV SP, R0");
    __asm(" BX LR");
}

//get service call number
uint32_t getSvcn()
{
    __asm(" LDR R1, [SP, #0x40]");            // move 16 registers (16*4 = 64d = 40h) to get PC address
    __asm(" LDR R0, [R1, #-0x02]!");          // fetch PC value
    __asm(" AND R0, #0x000000FF");            // get LSB value of PC
    __asm(" BX LR");
}

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
    NVIC_ST_RELOAD_R = 40000-1;                               // 25ns(system clock period)*40000 = 1ms
    NVIC_ST_CTRL_R = NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;
}

void rtosStart()
{
    // REQUIRED: add code to call the first task to be run
    _fn fn;
    taskCurrent = rtosScheduler();
    // Add code to initialize the SP with tcb[task_current].sp;
    systemSp = getSp();
    setSp(tcb[taskCurrent].sp);
    fn = (_fn) tcb[taskCurrent].pid;
    tcb[taskCurrent].state = STATE_READY;
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter for the first time to calculate task runtime
    (*fn)();                                        // running first task
}

bool createThread(_fn fn, char name[], int priority)
{
    __asm(" SVC #06");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm(" SVC #05");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t i = 0;
    while (i < MAX_TASKS)
    {
        if (tcb[i].pid == fn)
        {
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            break;
        }
        i++;
    }
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
    // push registers, call scheduler, pop registers, return to new function
    __asm(" SVC #01");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    // push registers, set state to delayed, store timeout, call scheduler, pop registers,
    // return to new function (separate unrun or ready processing)
    __asm(" SVC #02");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm(" SVC #03");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #04");
}

// REQUIRED: Implement prioritization to 8 levels
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
        if (priority)
        {
            if (tcb[task].skipCount >= tcb[task].currentPriority)
            {
                tcb[task].skipCount = 0;         // run task if max skip is reached, otherwise increase skip count
                ok = (tcb[task].state == STATE_READY | tcb[task].state == STATE_UNRUN);
            }
            else
                tcb[task].skipCount++;
        }
        else
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
    static uint16_t systickCount = 0;

    // for sleep
    for (i = 0; i < MAX_TASKS; i++)
    {
        if(tcb[i].state != STATE_INVALID && tcb[i].state == STATE_DELAYED)
        {
            tcb[i].ticks--;
            if(tcb[i].ticks == 0)
                tcb[i].state = STATE_READY;
        }
    }
    // IIR filter for CPU percentage
    if (systickCount == 100)                                // filtering after every 100ms
    {
        for (i = 0; i < MAX_TASKS; i++)
        {
           tcb[i].runtimeIir = tcb[i].runtimeIir * 9/10 + tcb[i].runtime / 10;   // Si = alpha * Si + (1-alpha) * Ti, where alpha=0.9
        }
        timePeriod = timePeriod * 9/10 + totalTime / 10;    // saving time period (IIR Filtering)

        // resetting time values after filtering
        for (i = 0; i < MAX_TASKS; i++)
        {
            tcb[i].runtime = 0;
        }
        totalTime = 0;
        systickCount = 0;
    }
    systickCount++;

    // preemptive mode
    if (preemptive)
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;           // switch tasks
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    // compiler pushes R2, R3, R4, LR (4 registers)
    // hardware pushes 9 registers
    uint16_t i;
    __asm(" PUSH {R4-R11}");                    // user pushes 8 registers, total 21 registers being pushed

    tcb[taskCurrent].sp = getSp();
    tcb[taskCurrent].runtime += TIMER1_TAV_R;   // counting task runtime before switching
    totalTime += TIMER1_TAV_R;

    if (taskCount > 0)
    {
        taskCurrent = rtosScheduler();
        TIMER1_TAV_R = 0;                       // resetting timer1 for counting new task runtime
    }

    if (tcb[taskCurrent].state == STATE_UNRUN)
    {
        // setting stack for unrun task
        for (i = 255; i >= 236; i--)
        {
            if (i == 255)
                stack[taskCurrent][i] = 0x01000000;                         // set XPSR
            else if (i == 254)
                stack[taskCurrent][i] = (uint32_t*) tcb[taskCurrent].pid;   // set PC
            else if (i == 247)
                stack[taskCurrent][i] =  0xFFFFFFF9;                        // set LR
            else
                stack[taskCurrent][i] = i;                                  // putting arbitrary values in other stack locations
        }
        tcb[taskCurrent].sp = &stack[taskCurrent][236];   // (236-1)+ 8(user pushed)+ 4(compiler pushed) = 247 to set SP to LR on return
        tcb[taskCurrent].state = STATE_READY;
    }
    setSp(tcb[taskCurrent].sp);
    __asm(" POP {R4-R11}");
    // compiler pops R2, R3, R4, PC
}

// function for getting R0 register value
uint32_t returnValue()
{
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint32_t R0 = returnValue();
    __asm(" MOV R0, R1");
    uint32_t R1 = returnValue();
    __asm(" MOV R0, R2");
    uint32_t R2 = returnValue();

    uint8_t destroyTask, containedSemaphore;
    uint8_t n, i, j = 0;
    bool  modifySemaphore = false;
    bool ok = false;
    bool found = false;
    n = getSvcn();                  // get service call number

    switch(n)
    {
    // for yield function
    case 1:
        tcb[taskCurrent].state = STATE_READY;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    // for sleep function
    case 2:
        tcb[taskCurrent].ticks = R0;
        tcb[taskCurrent].state = STATE_DELAYED;
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    // for wait function
    case 3:
        tempSemaphore = (struct semaphore*) R0;
        // if semaphore not available, block task and put into queue
        if (tempSemaphore->count == 0)
        {
            tempSemaphore->processQueue[tempSemaphore->queueSize++] = (uint32_t*) tcb[taskCurrent].pid;
            tcb[taskCurrent].state = STATE_BLOCKED;

            // priority inheritance
            if (pi)
            {
                for (i = 0; i < MAX_TASKS; i++)
                {
                   if(tcb[i].semaphore == tempSemaphore)
                   {
                       taskSemaphore = i;                   // find which task is holding semaphore
                       break;
                   }
                }
                // if the task holding semaphore has lower priority then elevate its priority to current task's priority
                if (tcb[taskSemaphore].currentPriority > tcb[taskCurrent].currentPriority)
                {
                    tcb[taskSemaphore].currentPriority = tcb[taskCurrent].currentPriority;
                }
            }
            tcb[taskCurrent].semaphore = tempSemaphore;     // current task waiting in semaphore
        }
        else
            tempSemaphore->count--;

        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    // for post function
    case 4:
        tempSemaphore = (struct semaphore*) R0;
        tempSemaphore->count++;
        // releasing task from queue
        if (tempSemaphore->queueSize > 0)
        {
            for (i = 0; i < MAX_TASKS; i++)
            {
                if(tempSemaphore->processQueue[0] == (uint32_t*) tcb[i].pid)
                {
                    tcb[i].state = STATE_READY;
                    break;
                }
            }
            // restore previously elevated task's priority
            tcb[taskSemaphore].currentPriority = tcb[taskSemaphore].priority;
            tempSemaphore->queueSize--;
            tempSemaphore->count--;
            // reorder queue elements
            while (j < tempSemaphore->queueSize)
            {
                tempSemaphore->processQueue[j++] = tempSemaphore->processQueue[j+1];
            }
        }
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    // for destroyThread function
    case 5:
        tempThread = (_fn)R0;
        for (i = 0; i < MAX_TASKS; i++)
        {
           if (tcb[i].pid == tempThread)
           {
               destroyTask = i;                 // finding which thread to destroy
               found = true;
               break;
           }
        }
        if(found)
        {
            tcb[destroyTask].state = STATE_INVALID;
            tcb[destroyTask].pid = 0;
            tcb[destroyTask].runtimeIir = 0;
            taskCount--;

            for (i = 255; i >= 236; i--)
            {
                stack[destroyTask][i] = 0;          // clearing stack of destroyed task
            }

            for (i = 0; i < MAX_SEMAPHORES; i++)
            {
               if(&semaphores[i] == tcb[destroyTask].semaphore)
               {
                   containedSemaphore = i;          // finding which semaphore destroyed task contains
                   modifySemaphore = true;
                   break;
               }
            }

            if (modifySemaphore)
            {
                for (i = 0; i < semaphores[containedSemaphore].queueSize; i++)
                {
                    // find if destroyed task is in semaphore queue
                    if (semaphores[containedSemaphore].processQueue[i] == (uint32_t*) tcb[destroyTask].pid)
                    {
                        // remove task from queue and reorder queue elements
                        for (j = i; j < semaphores[containedSemaphore].queueSize; j++)
                        {
                            semaphores[containedSemaphore].processQueue[j] = semaphores[containedSemaphore].processQueue[j+1];
                        }
                        semaphores[containedSemaphore].queueSize--;
                    }
                }
                tcb[destroyTask].semaphore = 0;
            }
        }
        if (!found)
            putsUart0("Thread already deleted\r\n");

        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        break;

    // for createThread function
    case 6:
        i = 0;
        // REQUIRED: store the thread name
        // add task if room in task list
        if (taskCount < MAX_TASKS)
        {
            // make sure fn not already in list (prevent reentrancy)
            while (!found && (i < MAX_TASKS))
            {
                found = (tcb[i++].pid == (_fn)R0);
            }
            if (!found)
            {
                // find first available tcb record
                i = 0;
                while (tcb[i].state != STATE_INVALID) {i++;}
                tcb[i].state = STATE_UNRUN;
                tcb[i].pid = (_fn)R0;
                tcb[i].sp = &stack[i][255];
                tcb[i].priority = R2;
                tcb[i].currentPriority = R2;
                tcb[i].skipCount = 0;
                strCopy(tcb[i].name, R1);
                // increment task count
                taskCount++;
                createThreadTable(i);       // create directory entry for newly created task
                ok = true;
            }
        }
        break;
        // REQUIRED: allow tasks switches again
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
    //           5 pushbuttons, and uart

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A, B and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOF;

    // Configure on board blue LED
    GPIO_PORTF_DIR_R |= 0x04;  // set bit 2 as output
    GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= 0x04;  // enable blue LED

    // Configure off board orange, red, green, and yellow LEDs and 5 push buttons
    GPIO_PORTA_DIR_R |= 0xE0;  // set bit 5,6,7 as output
    GPIO_PORTA_DR2R_R |= 0xE0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R |= 0xFC;  // enable digital at pin 2,3,4,5,6,7
    GPIO_PORTA_PUR_R |= 0x1C;  // for push buttons at pin 2,3,4

    GPIO_PORTB_DIR_R |= 0x10;  // set bit 4 as output
    GPIO_PORTB_DR2R_R |= 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTB_DEN_R |= 0xD0;  // enable digital at pin 4,6,7
    GPIO_PORTB_PUR_R |= 0xC0;  // for push buttons at pin 6,7

    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
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

    // Configure Timer 1 to count runtime
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;      // turn-on timer
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;;         // configure as 32-bit counter (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    TIMER1_TAV_R = 0;                               // zero counter for first period
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
    // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    if (PB0 == 0) return 1;
    else if (PB1 == 0) return 2;
    else if (PB2 == 0) return 4;
    else if (PB3 == 0) return 8;
    else if (PB4 == 0) return 16;
    else return 0;
}

// custom function calculating the length of string
uint8_t strLength(char *str)
{
    uint8_t i=0;
    while (*str++) i++;
    return i;
}

// custom function to copy string from str2 to str1
void strCopy(char *str1, char *str2)
{
    while (*str2) *str1++ = *str2++;
    *str1 = '\0';
}

// function returns true if two strings are equal
bool strEqual(char* str1, char *str2)
{
    while (*str1 || *str2)
    {
        if (*str1++ != *str2++)
            return false;
    }
    return true;
}

// function returns lowercase of string
char* strLower(char* str)
{
    uint8_t i = 0;
    uint8_t j = strLength(str);
    while (i < j)
        str[i++] = tolower(str[i]);
    return str;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i, j;
    j = strLength(str);
    for (i = 0; i < j; i++)
        putcUart0(str[i]);
}

// Write string in 12 bit alignment
void putsAlignedUart0(char* str)
{
    uint8_t i, j;
    j = strLength(str);
    for (i = 0; i < j; i++)
        putcUart0(str[i]);
    while (12 - i)
    {
        putcUart0(' ');
        i++;
    }
}

// Write string in 15 bit alignment
void putsAligned15Uart0(char* str)
{
    uint8_t i, j;
    j = strLength(str);
    for (i = 0; i < j; i++)
        putcUart0(str[i]);
    while (15 - i)
    {
        putcUart0(' ');
        i++;
    }
}

//  Blocking function that writes number in UART as string
void putnumUart0(uint32_t n)
{
    char str[MAX_CHARS];
    ltoa(n, str);
    putsAlignedUart0(str);
}

//  Blocking function that writes floating number in UART as string
void putfloatUart0(uint32_t n)
{
    char str[6];
    uint32_t rem;
    uint8_t i = 5;
    while (i != 0)
    {
        i--;
        rem = n % 10;
        if (i == 2)
            str[i] = '.';
        else
        {
            str[i] = '0' + rem;
            n /= 10;
        }
    }
    str[5] = '\0';
    putsUart0(str);
}

// Blocking function that writes number in UART as 8 digit hex (0x00000000) string
void puthexUart0(uint32_t n)
{
    char str[11];
    uint32_t rem;
    uint8_t i = 9;
    str[0] = '0';
    str[1] = 'x';
    while (i != 1)
    {
        rem = n % 16;
        str[i--] = (rem > 9) ? ('A' + rem - 10) : ('0' + rem);
        n /= 16;
    }
    str[10] = '\0';
    putsAlignedUart0(str);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE) yield();
    return UART0_DR_R & 0xFF;
}

uint32_t getNumber(uint8_t i)
{
    return atoi(&strInput[fieldPosition[i]]);
}

char* getString(uint8_t i)
{
    return &strInput[fieldPosition[i]];
}

bool isCommand(char* verb, uint8_t minArgs)
{
    if(strEqual(getString(0),verb) && minArgs<=(fieldCount-1)) return true;
    else return false;
}

// Blocking function that takes input command from the user
void inputCommand()
{
    uint8_t flag;
    uint8_t count=0;
    while (1)
    {
        flag = 1;
        char c = getcUart0();

        // Take input until enter pressed or max character limit is reached
        while (count<MAX_CHARS && c!=13)
        {
            flag = 0;
            if (c == 127 || c == 8) //  destructive backspace, For putty c==127, for teraterm c==8
            {
                if (count > 0)
                {
                    count--;
                    putcUart0(c);
                }
                break;
            }
            if (c < 32) break;
            strInput[count++] = tolower(c);
            putcUart0(c);
            break;
        }

        // Put null at the end of strInput
        if (flag == 1)
        {
            strInput[count]=0;
            break;
        }
    }
}

// function for tokenizing input command
void strTok(char* str)
{
    uint8_t i;
    uint8_t count = strLength(str);
    fieldCount = 0;

    // Check if alpha is at the beginning: Field position starts from the beginning
    if (str[0] > 96 && str[0] < 123)
    {
        fieldPosition[fieldCount] = 0;
        fieldType[fieldCount++] = 'a';
    }

    // Find field entry
    for (i = 0; i < count; i++)
    {
        // if delimiter excluding '&' and '.'
        if ((str[i] < 48 && str[i] != 38 && str[i] != 46) || (str[i] > 57 && str[i] < 97) || str[i] > 122)
        {
            str[i] = 0;  // Convert delimiter to NULL

            // Delimiter to alpha transition
            if (str[i+1] > 96 && str[i+1] < 123)
            {
                fieldPosition[fieldCount] = i+1;
                fieldType[fieldCount++] = 'a';
            }

            // Delimiter to number transition
            if (str[i+1] != 47 && str[i+1] > 45 && str[i+1] < 58)
            {
                fieldPosition[fieldCount] = i+1;
                fieldType[fieldCount++] = 'n';
            }

            // Delimiter to '&' symbol transition
            if (str[i+1] == 38)
            {
                fieldPosition[fieldCount] = i+1;
                fieldType[fieldCount++] = 's';
            }
        }

        //Alpha to '&' symbol transition
        if (str[i] > 96 && str[i] < 123)
        {
            if (str[i+1] == 38)
            {
                str[i+1] = 0;
                str[i+2] = 38;
                fieldPosition[fieldCount] = i+2;
                fieldType[fieldCount++] = 's';
            }
        }
    }
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
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

/*void idle2()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}*/

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
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(1000);
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
    uint32_t pidNumber, i, j, *k, *l, m;

    while (true)
    {
        // REQUIRED: add processing for the shell commands through the UART here
        // clearing previous input string
        for (i = 0; i < MAX_CHARS; i++)
            strInput[i] = 0;

        bool processed = false;
        bool found = false;

        // Wait for user input
        putsUart0(".....................................................................................\r\n");
        inputCommand();
        putsUart0("\r\n");
        putsUart0(".....................................................................................\r\n");

        // Tokenize user input
        strTok(strInput);

        // display PID number
        if (isCommand("pidof", 1))
        {
            for (i = 0; i < MAX_TASKS; i++)
            {
                if (strEqual(strLower(thread[i].name), getString(1)))
                {
                    pidNumber = (uint32_t)thread[i].pid;
                    putnumUart0(pidNumber);
                    putsUart0("\r\n");
                    found = true;
                    break;
                }
            }
            if (!found)
                putsUart0("Invalid Process Name\r\n");

            processed = true;
            putsUart0("\r\n");
            putsUart0("\r\n");
        }

        // IPCS
        if (isCommand("ipcs", 0))
        {
            putsUart0("List of Semaphores:\r\n\r\n");
            for (i = 0; i < semaphoreCount; i++)
            {
                // semaphore name
                if (i == 0) putsUart0("Semaphore:keyPressed\r\n");
                else if (i == 1) putsUart0("Semaphore:keyReleased\r\n");
                else if (i == 2) putsUart0("Semaphore:flashReq\r\n");
                else if (i == 3) putsUart0("Semaphore:resource\r\n");
                // number of tasks in queue
                putsUart0("Queue Count: ");
                putnumUart0(semaphores[i].queueSize);
                putsUart0("\r\n");
                // name of task waiting in queue
                putsUart0("Queue Waiters: ");
                j = 0;
                while (j < semaphores[i].queueSize)
                {
                    putnumUart0(semaphores[i].processQueue[j++]);
                    for (m = 0; m < MAX_TASKS; m++)
                    {
                        if (tcb[m].pid == semaphores[i].processQueue[j-1])
                        {
                            putsUart0("(");
                            putsUart0(tcb[m].name);
                            putsUart0(")");
                        }
                    }
                    putsUart0("  ");
                }
                putsUart0("\r\n");
                // semaphore count number
                putsUart0("Count at Semaphore:  ");
                putnumUart0(semaphores[i].count);
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            processed = true;
            putsUart0("\r\n");
        }

        // PS
        if (isCommand("ps", 0))
        {
            putsUart0(" PID       Thread Name       State                       CPU Percentage\r\n");
            putsUart0("...................................................................................\r\n");
            for (i = 0; i < MAX_TASKS; i++)
            {
                while (tcb[i].state == STATE_INVALID) i++;               // skip if task is not valid
                if (i >= MAX_TASKS) break;
                // PID Number
                putnumUart0((uint32_t*)tcb[i].pid);
                // Thread Name
                putsAlignedUart0(tcb[i].name);
                putsUart0("     ");
                // State
                if (tcb[i].state == 1)
                {
                    putsAlignedUart0("UNRUN");
                    putsAligned15Uart0(" ");
                }
                else if (tcb[i].state == 2)
                {
                    putsAlignedUart0("READY");
                    putsAligned15Uart0(" ");
                }
                else if (tcb[i].state == 3)
                {
                    putsAlignedUart0("BLOCKED");
                    for (j = 0; j < MAX_SEMAPHORES; j++)
                    {
                       if (&semaphores[j] == tcb[i].semaphore)
                       {
                           if (j == 0) putsAligned15Uart0("by keyPressed");
                           else if (j == 1) putsAligned15Uart0("by keyReleased");
                           else if (j == 2) putsAligned15Uart0("by flashReq");
                           else if (j == 3) putsAligned15Uart0("by resource");
                           break;
                       }
                    }
                }
                else if (tcb[i].state == 4)
                {
                    putsAlignedUart0("DELAYED");
                    putsAligned15Uart0(" ");
                }
                putsUart0("     ");
                // CPU Percentage
                putfloatUart0((tcb[i].runtimeIir * 10000) / timePeriod); // Calculate in 0.01%
                putsUart0("\r\n");
            }
            putsUart0("\r\n");
            putsUart0("\r\n");

            // display stack in hex value for all valid threads
            putsUart0("Stack Values for All Valid Threads:\r\n");
            putsUart0("\r\n");
            for (i = 0; i < MAX_TASKS; i++)
            {
                while (tcb[i].state == STATE_INVALID) i++;      // skip if task is not valid
                if (i >= MAX_TASKS) break;
                putsUart0("Thread Name:");
                putsUart0(tcb[i].name);
                putsUart0("\r\n");
                k = &stack[i][255];
                l = tcb[i].sp;
                j = (k -l);                                     // number of items in stack
                m = 1;
                while (j)
                {
                    puthexUart0(stack[i][255-j]);
                    if ((m++ % 6) == 0) putsUart0("\r\n");      // skip to next row (put 6 items in a row)
                    j--;
                }
                putsUart0("\r\n");
                putsUart0("\r\n");
            }
            processed = true;
            putsUart0("\r\n");
        }

        // destroy thread
        if (isCommand("kill", 1))
        {
            for (i = 0; i < MAX_TASKS; i++)
            {
                if (getNumber(1) == (uint32_t*) tcb[i].pid)
                {
                    destroyThread((_fn)tcb[i].pid);
                    putsUart0("Process Killed\r\n");
                    found = true;
                    break;
                }
            }
            if (!found)
                putsUart0("Invalid PID\r\n");

            processed = true;
            putsUart0("\r\n");
            putsUart0("\r\n");
        }

        // create thread
        if (strEqual(getString(1),"&"))
        {
            for (i = 0; i < MAX_TASKS; i++)
            {
                if (strEqual(strLower(thread[i].name), getString(0)))
                {
                    if ((uint32_t*) tcb[i].pid != 0)
                        putsUart0("Thread Already Running\r\n");
                    else
                    {
                        createThread((_fn)thread[i].pid, thread[i].name, thread[i].priority);
                        putsUart0("Thread Created\r\n");
                    }
                    processed = true;
                    break;
                }
            }
            putsUart0("\r\n");
            putsUart0("\r\n");
        }

        // set command
        if (isCommand("set", 2))
        {
            // priority on/off
            if (strEqual(getString(1), "priority"))
            {
                if (strEqual(getString(2), "on"))
                {
                    priority = true;
                    processed = true;
                }
                else if (strEqual(getString(2), "off"))
                {
                    priority = false;
                    processed = true;
                }
            }

            // preemptive on/off
            else if (strEqual(getString(1), "preemptive"))
            {
                if (strEqual(getString(2), "on"))
                {
                    preemptive = true;
                    processed = true;
                }
                else if (strEqual(getString(2), "off"))
                {
                    preemptive = false;
                    processed = true;
                }
            }

            // pi (priority inheritance) on/off
            else if (strEqual(getString(1), "pi"))
            {
                if (strEqual(getString(2), "on"))
                {
                    pi = true;
                    processed = true;
                }
                else if (strEqual(getString(2), "off"))
                {
                    pi = false;
                    processed = true;
                }
            }
            putsUart0("\r\n");
        }

        // reboot
        if (isCommand("reboot",0))
        {
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;  // core reset: returns to reset isr, restarts program counter
            processed = true;
        }

        if (!processed)
        {
            putsUart0("ERROR: INVALID INPUT");
            putsUart0("\r\n");
            putsUart0("\r\n");
        }
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

    putsUart0("\r\n");
    putsUart0("Project RTOS\r\n");
    putsUart0("Author: Sujjatul Islam\r\n\r\n");

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
    ok =  createThread(idle, "Idle", 15);

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 12);
    ok &= createThread(flash4Hz, "Flash4Hz", 4);
    ok &= createThread(oneshot, "OneShot", 4);
    ok &= createThread(readKeys, "ReadKeys", 12);
    ok &= createThread(debounce, "Debounce", 12);
    ok &= createThread(important, "Important", 0);
    ok &= createThread(uncooperative, "Uncoop", 10);
    ok &= createThread(shell, "Shell", 8);

    // Start up RTOS
    if (ok)
        rtosStart(); // never returns
    else
        RED_LED = 1;

    return 0;
}
