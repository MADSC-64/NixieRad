/** Generated by itemis CREATE code generator. */


#include "sc_types.h"

#include "Statechart.h"
#include "Statechart_required.h"

/*! \file
Implementation of the state machine 'Statechart'
*/

#ifndef SC_UNUSED
#define SC_UNUSED(P) (void)(P)
#endif

/* prototypes of all internal functions */
static void enact_s_Fast_Measurement____(Statechart* handle);
static void enact_s_Slow_Measurement____(Statechart* handle);
static void enact_s_msV_Rate(Statechart* handle);
static void enact_s_Threshold(Statechart* handle);
static void enseq_s_Fast_Measurement_____default(Statechart* handle);
static void enseq_s_Slow_Measurement_____default(Statechart* handle);
static void enseq_s_msV_Rate_default(Statechart* handle);
static void enseq_s_Threshold_default(Statechart* handle);
static void enseq_s_default(Statechart* handle);
static void exseq_s_Fast_Measurement____(Statechart* handle);
static void exseq_s_Slow_Measurement____(Statechart* handle);
static void exseq_s_msV_Rate(Statechart* handle);
static void exseq_s_Threshold(Statechart* handle);
static void exseq_s(Statechart* handle);
static void react_s__entry_Default(Statechart* handle);

/*! State machine reactions. */
static sc_integer react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Fast Measurement    . */
static sc_integer s_Fast_Measurement_____react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Slow Measurement    . */
static sc_integer s_Slow_Measurement_____react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state msV Rate. */
static sc_integer s_msV_Rate_react(Statechart* handle, const sc_integer transitioned_before);

/*! The reactions of state Threshold. */
static sc_integer s_Threshold_react(Statechart* handle, const sc_integer transitioned_before);


static void clear_in_events(Statechart* handle);

static void micro_step(Statechart* handle);

/*! Performs a 'run to completion' step. */
static void run_cycle(Statechart* handle);






static void statechart_eventqueue_init(statechart_eventqueue * eq, statechart_event *buffer, sc_integer capacity);
static sc_integer statechart_eventqueue_size(statechart_eventqueue * eq);
static void statechart_event_init(statechart_event * ev, StatechartEventID name);
static statechart_event statechart_eventqueue_pop(statechart_eventqueue * eq);
static sc_boolean statechart_eventqueue_push(statechart_eventqueue * eq, statechart_event ev);
static void statechart_add_event_to_queue(statechart_eventqueue * eq, StatechartEventID name);
static sc_boolean statechart_dispatch_event(Statechart* handle, const statechart_event * event);
static statechart_event statechart_get_next_event(Statechart* handle);
static sc_boolean statechart_dispatch_next_event(Statechart* handle);


void statechart_init(Statechart* handle)
{
	sc_integer i;
	
	for (i = 0; i < STATECHART_MAX_ORTHOGONAL_STATES; ++i)
	{
		handle->stateConfVector[i] = Statechart_last_state;
	}
	
				
	clear_in_events(handle);
	
	
	
	handle->isExecuting = bool_false;
	statechart_eventqueue_init(&handle->in_event_queue, handle->in_buffer, STATECHART_IN_EVENTQUEUE_BUFFERSIZE);
}

void statechart_enter(Statechart* handle)
{
	/* Activates the state machine. */
	if (handle->isExecuting == bool_true)
	{ 
		return;
	} 
	handle->isExecuting = bool_true;
	/* Default enter sequence for statechart Statechart */
	enseq_s_default(handle);
	handle->doCompletion = bool_false;
	do
	{ 
		if (handle->completed == bool_true)
		{ 
			handle->doCompletion = bool_true;
		} 
		handle->completed = bool_false;
		micro_step(handle);
		handle->doCompletion = bool_false;
	} while (handle->completed == bool_true);
	handle->isExecuting = bool_false;
}

void statechart_exit(Statechart* handle)
{
	/* Deactivates the state machine. */
	if (handle->isExecuting == bool_true)
	{ 
		return;
	} 
	handle->isExecuting = bool_true;
	/* Default exit sequence for statechart Statechart */
	exseq_s(handle);
	handle->isExecuting = bool_false;
}

/*!
Can be used by the client code to trigger a run to completion step without raising an event.
*/
void statechart_trigger_without_event(Statechart* handle) {
	run_cycle(handle);
}


sc_boolean statechart_is_active(const Statechart* handle)
{
	sc_boolean result = bool_false;
	sc_integer i;
	
	for(i = 0; i < STATECHART_MAX_ORTHOGONAL_STATES; i++)
	{
		result = result || handle->stateConfVector[i] != Statechart_last_state;
	}
	
	return result;
}

/* 
 * Always returns 'false' since this state machine can never become final.
 */
sc_boolean statechart_is_final(const Statechart* handle)
{
	SC_UNUSED(handle);
	return bool_false;
}

sc_boolean statechart_is_state_active(const Statechart* handle, StatechartStates state)
{
	sc_boolean result = bool_false;
	switch (state)
	{
		case Statechart_s_Fast_Measurement____ :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_S_FAST_MEASUREMENT____] == Statechart_s_Fast_Measurement____
			);
				break;
		case Statechart_s_Slow_Measurement____ :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_S_SLOW_MEASUREMENT____] == Statechart_s_Slow_Measurement____
			);
				break;
		case Statechart_s_msV_Rate :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_S_MSV_RATE] == Statechart_s_msV_Rate
			);
				break;
		case Statechart_s_Threshold :
			result = (sc_boolean) (handle->stateConfVector[SCVI_STATECHART_S_THRESHOLD] == Statechart_s_Threshold
			);
				break;
			default:
				result = bool_false;
				break;
		}
		return result;
	}

static void clear_in_events(Statechart* handle)
{
	handle->iface.BeSlow_raised = bool_false;
	handle->iface.BeFast_raised = bool_false;
	handle->iface.setmsVRate_raised = bool_false;
	handle->iface.setThreshold_raised = bool_false;
}

static void micro_step(Statechart* handle)
{
	switch(handle->stateConfVector[ 0 ])
	{
		case Statechart_s_Fast_Measurement____ :
		{
			s_Fast_Measurement_____react(handle,-1);
			break;
		}
		case Statechart_s_Slow_Measurement____ :
		{
			s_Slow_Measurement_____react(handle,-1);
			break;
		}
		case Statechart_s_msV_Rate :
		{
			s_msV_Rate_react(handle,-1);
			break;
		}
		case Statechart_s_Threshold :
		{
			s_Threshold_react(handle,-1);
			break;
		}
		default: 
			/* do nothing */
			break;
	}
}

static void run_cycle(Statechart* handle)
{
	/* Performs a 'run to completion' step. */
	if (handle->isExecuting == bool_true)
	{ 
		return;
	} 
	handle->isExecuting = bool_true;
	statechart_dispatch_next_event(handle);
	do
	{ 
		handle->doCompletion = bool_false;
		do
		{ 
			if (handle->completed == bool_true)
			{ 
				handle->doCompletion = bool_true;
			} 
			handle->completed = bool_false;
			micro_step(handle);
			handle->doCompletion = bool_false;
		} while (handle->completed == bool_true);
		clear_in_events(handle);
	} while (statechart_dispatch_next_event(handle) == bool_true);
	handle->isExecuting = bool_false;
}


void statechart_raise_beSlow(Statechart* handle)
{
	statechart_add_event_to_queue(&(handle->in_event_queue), Statechart_BeSlow);
	run_cycle(handle);
}

void statechart_raise_beFast(Statechart* handle)
{
	statechart_add_event_to_queue(&(handle->in_event_queue), Statechart_BeFast);
	run_cycle(handle);
}

void statechart_raise_setmsVRate(Statechart* handle)
{
	statechart_add_event_to_queue(&(handle->in_event_queue), Statechart_setmsVRate);
	run_cycle(handle);
}

void statechart_raise_setThreshold(Statechart* handle)
{
	statechart_add_event_to_queue(&(handle->in_event_queue), Statechart_setThreshold);
	run_cycle(handle);
}






/* implementations of all internal functions */

/* Entry action for state 'Fast Measurement    '. */
static void enact_s_Fast_Measurement____(Statechart* handle)
{
	/* Entry action for state 'Fast Measurement    '. */
	statechart_takeFastMeasurement(handle);
}

/* Entry action for state 'Slow Measurement    '. */
static void enact_s_Slow_Measurement____(Statechart* handle)
{
	/* Entry action for state 'Slow Measurement    '. */
	statechart_takeSlowMeasurement(handle);
}

static void enact_s_msV_Rate(Statechart* handle)
{
	/* Entry action for state 'msV Rate'. */
	statechart_changemsVRate(handle);
	handle->completed = bool_true;
}

static void enact_s_Threshold(Statechart* handle)
{
	/* Entry action for state 'Threshold'. */
	statechart_changeThreshold(handle);
	handle->completed = bool_true;
}

/* 'default' enter sequence for state Fast Measurement     */
static void enseq_s_Fast_Measurement_____default(Statechart* handle)
{
	/* 'default' enter sequence for state Fast Measurement     */
	enact_s_Fast_Measurement____(handle);
	handle->stateConfVector[0] = Statechart_s_Fast_Measurement____;
}

/* 'default' enter sequence for state Slow Measurement     */
static void enseq_s_Slow_Measurement_____default(Statechart* handle)
{
	/* 'default' enter sequence for state Slow Measurement     */
	enact_s_Slow_Measurement____(handle);
	handle->stateConfVector[0] = Statechart_s_Slow_Measurement____;
}

/* 'default' enter sequence for state msV Rate */
static void enseq_s_msV_Rate_default(Statechart* handle)
{
	/* 'default' enter sequence for state msV Rate */
	enact_s_msV_Rate(handle);
	handle->stateConfVector[0] = Statechart_s_msV_Rate;
}

/* 'default' enter sequence for state Threshold */
static void enseq_s_Threshold_default(Statechart* handle)
{
	/* 'default' enter sequence for state Threshold */
	enact_s_Threshold(handle);
	handle->stateConfVector[0] = Statechart_s_Threshold;
}

/* 'default' enter sequence for region s */
static void enseq_s_default(Statechart* handle)
{
	/* 'default' enter sequence for region s */
	react_s__entry_Default(handle);
}

/* Default exit sequence for state Fast Measurement     */
static void exseq_s_Fast_Measurement____(Statechart* handle)
{
	/* Default exit sequence for state Fast Measurement     */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state Slow Measurement     */
static void exseq_s_Slow_Measurement____(Statechart* handle)
{
	/* Default exit sequence for state Slow Measurement     */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state msV Rate */
static void exseq_s_msV_Rate(Statechart* handle)
{
	/* Default exit sequence for state msV Rate */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for state Threshold */
static void exseq_s_Threshold(Statechart* handle)
{
	/* Default exit sequence for state Threshold */
	handle->stateConfVector[0] = Statechart_last_state;
}

/* Default exit sequence for region s */
static void exseq_s(Statechart* handle)
{
	/* Default exit sequence for region s */
	/* Handle exit of all possible states (of Statechart.s) at position 0... */
	switch(handle->stateConfVector[ 0 ])
	{
		case Statechart_s_Fast_Measurement____ :
		{
			exseq_s_Fast_Measurement____(handle);
			break;
		}
		case Statechart_s_Slow_Measurement____ :
		{
			exseq_s_Slow_Measurement____(handle);
			break;
		}
		case Statechart_s_msV_Rate :
		{
			exseq_s_msV_Rate(handle);
			break;
		}
		case Statechart_s_Threshold :
		{
			exseq_s_Threshold(handle);
			break;
		}
		default: 
			/* do nothing */
			break;
	}
}

/* Default react sequence for initial entry  */
static void react_s__entry_Default(Statechart* handle)
{
	/* Default react sequence for initial entry  */
	enseq_s_Fast_Measurement_____default(handle);
}


static sc_integer react(Statechart* handle, const sc_integer transitioned_before)
{
	/* State machine reactions. */
	SC_UNUSED(handle);
	return transitioned_before;
}

static sc_integer s_Fast_Measurement_____react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Fast Measurement    . */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_false)
	{ 
		if ((transitioned_after) < (0))
		{ 
			if (handle->iface.BeSlow_raised == bool_true)
			{ 
				exseq_s_Fast_Measurement____(handle);
				enseq_s_Slow_Measurement_____default(handle);
				react(handle,0);
				transitioned_after = 0;
			}  else
			{
				if (handle->iface.setThreshold_raised == bool_true)
				{ 
					exseq_s_Fast_Measurement____(handle);
					enseq_s_Threshold_default(handle);
					react(handle,0);
					transitioned_after = 0;
				}  else
				{
					if (handle->iface.setmsVRate_raised == bool_true)
					{ 
						exseq_s_Fast_Measurement____(handle);
						enseq_s_msV_Rate_default(handle);
						react(handle,0);
						transitioned_after = 0;
					} 
				}
			}
		} 
		/* If no transition was taken */
		if ((transitioned_after) == (transitioned_before))
		{ 
			/* then execute local reactions. */
			transitioned_after = react(handle,transitioned_before);
		} 
	} return transitioned_after;
}

static sc_integer s_Slow_Measurement_____react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Slow Measurement    . */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_false)
	{ 
		if ((transitioned_after) < (0))
		{ 
			if (handle->iface.BeFast_raised == bool_true)
			{ 
				exseq_s_Slow_Measurement____(handle);
				enseq_s_Fast_Measurement_____default(handle);
				react(handle,0);
				transitioned_after = 0;
			}  else
			{
				if (handle->iface.setmsVRate_raised == bool_true)
				{ 
					exseq_s_Slow_Measurement____(handle);
					enseq_s_msV_Rate_default(handle);
					react(handle,0);
					transitioned_after = 0;
				}  else
				{
					if (handle->iface.setThreshold_raised == bool_true)
					{ 
						exseq_s_Slow_Measurement____(handle);
						enseq_s_Threshold_default(handle);
						react(handle,0);
						transitioned_after = 0;
					} 
				}
			}
		} 
		/* If no transition was taken */
		if ((transitioned_after) == (transitioned_before))
		{ 
			/* then execute local reactions. */
			transitioned_after = react(handle,transitioned_before);
		} 
	} return transitioned_after;
}

static sc_integer s_msV_Rate_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state msV Rate. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state msV Rate */
		handle->stateConfVector[0] = Statechart_last_state;
		/* 'default' enter sequence for state Fast Measurement     */
		enact_s_Fast_Measurement____(handle);
		handle->stateConfVector[0] = Statechart_s_Fast_Measurement____;
		react(handle,0);
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = react(handle,transitioned_before);
	}
	return transitioned_after;
}

static sc_integer s_Threshold_react(Statechart* handle, const sc_integer transitioned_before)
{
	/* The reactions of state Threshold. */
 			sc_integer transitioned_after = transitioned_before;
	if (handle->doCompletion == bool_true)
	{ 
		/* Default exit sequence for state Threshold */
		handle->stateConfVector[0] = Statechart_last_state;
		/* 'default' enter sequence for state Fast Measurement     */
		enact_s_Fast_Measurement____(handle);
		handle->stateConfVector[0] = Statechart_s_Fast_Measurement____;
		react(handle,0);
	}  else
	{
		/* Always execute local reactions. */
		transitioned_after = react(handle,transitioned_before);
	}
	return transitioned_after;
}




static void statechart_eventqueue_init(statechart_eventqueue * eq, statechart_event *buffer, sc_integer capacity)
{
	eq->events = buffer;
	eq->capacity = capacity;
	eq->push_index = 0;
	eq->pop_index = 0;
	eq->size = 0;
}

static sc_integer statechart_eventqueue_size(statechart_eventqueue * eq)
{
	return eq->size;
}

static statechart_event statechart_eventqueue_pop(statechart_eventqueue * eq)
{
	statechart_event event;
	if(statechart_eventqueue_size(eq) <= 0) {
		statechart_event_init(&event, Statechart_invalid_event);
	}
	else {
		event = eq->events[eq->pop_index];
		
		if(eq->pop_index < eq->capacity - 1) {
			eq->pop_index++;
		} 
		else {
			eq->pop_index = 0;
		}
		eq->size--;
	}
	return event;
}
static sc_boolean statechart_eventqueue_push(statechart_eventqueue * eq, statechart_event ev)
{
	if(statechart_eventqueue_size(eq) == eq->capacity) {
		return bool_false;
	}
	else {
		eq->events[eq->push_index] = ev;
		
		if(eq->push_index < eq->capacity - 1) {
			eq->push_index++;
		}
		else {
			eq->push_index = 0;
		}
		eq->size++;
		
		return bool_true;
	}
}
static void statechart_event_init(statechart_event * ev, StatechartEventID name)
{
	ev->name = name;
}

static void statechart_add_event_to_queue(statechart_eventqueue * eq, StatechartEventID name)
{
	statechart_event event;
	statechart_event_init(&event, name);
	statechart_eventqueue_push(eq, event);
}

static sc_boolean statechart_dispatch_event(Statechart* handle, const statechart_event * event) {
	switch(event->name) {
		case Statechart_BeSlow:
		{
			handle->iface.BeSlow_raised = bool_true;
			return bool_true;
		}
		case Statechart_BeFast:
		{
			handle->iface.BeFast_raised = bool_true;
			return bool_true;
		}
		case Statechart_setmsVRate:
		{
			handle->iface.setmsVRate_raised = bool_true;
			return bool_true;
		}
		case Statechart_setThreshold:
		{
			handle->iface.setThreshold_raised = bool_true;
			return bool_true;
		}
		default:
			return bool_false;
	}
}

static statechart_event statechart_get_next_event(Statechart* handle)
{
	statechart_event next_event;
	statechart_event_init(&next_event, Statechart_invalid_event);
	if(statechart_eventqueue_size(&(handle->in_event_queue)) > 0) {
		next_event = statechart_eventqueue_pop(&(handle->in_event_queue));
	}
	return next_event;
}

static sc_boolean statechart_dispatch_next_event(Statechart* handle)
{
	statechart_event nextEvent;
	nextEvent = statechart_get_next_event(handle);
	return statechart_dispatch_event(handle, &nextEvent);
}
