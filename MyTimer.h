typedef struct _my_msecTIMER 
{
  bool TimerState;
  bool TimerSet;
  uint64_t TimerThreashold; 
} msec_TIMER;

typedef struct _my_usecTIMER 
{
  bool TimerState;
  bool TimerSet;
  uint64_t TimerThreashold; 
} usec_TIMER;

void TIMER_init(msec_TIMER *pTIMER)
{
  pTIMER->TimerState=0;
  pTIMER->TimerSet=0;
  pTIMER->TimerThreashold=0;
  return;
}
void TIMER_init(usec_TIMER *pTIMER)
{
  pTIMER->TimerState=0;
  pTIMER->TimerSet=0;
  pTIMER->TimerThreashold=0;
  return;
}

void TIMER_set(msec_TIMER *pTIMER, uint16_t msecInterval)
{
  if (!(pTIMER->TimerSet))
  {
    pTIMER->TimerState=0;
    pTIMER->TimerSet=1;
    pTIMER->TimerThreashold=millis()+msecInterval;
  }
  if (pTIMER->TimerThreashold<=millis())
    pTIMER->TimerState=1;
  return;
}
void TIMER_set(usec_TIMER *pTIMER, uint16_t usecInterval)
{
  if (!(pTIMER->TimerSet))
  {
    pTIMER->TimerState=0;
    pTIMER->TimerSet=1;
    pTIMER->TimerThreashold=micros()+usecInterval;
  }
  if (pTIMER->TimerThreashold<=micros())
    pTIMER->TimerState=1;
  return;
}


void TIMER_reset(msec_TIMER *pTIMER)
{
  pTIMER->TimerState=0;
  pTIMER->TimerSet=0;
  pTIMER->TimerThreashold=0;
  return;
}
void TIMER_reset(usec_TIMER *pTIMER)
{
  pTIMER->TimerState=0;
  pTIMER->TimerSet=0;
  pTIMER->TimerThreashold=0;
  return;
}

bool TIMER_state(msec_TIMER *pTIMER)
{
   return(pTIMER->TimerState && pTIMER->TimerSet);
}
bool TIMER_state(usec_TIMER *pTIMER)
{
   return(pTIMER->TimerState && pTIMER->TimerSet);
}
