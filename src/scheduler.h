// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

typedef void b2TaskCallback( void* taskContext );
typedef struct b2Scheduler b2Scheduler;

b2Scheduler* b2CreateScheduler( int workerCount );
void b2DestroyScheduler( b2Scheduler* scheduler );
void b2ResetScheduler( b2Scheduler* scheduler );

// See b2EnqueueTaskCallback and b2FinishTaskCallback
void* b2SchedulerEnqueueTask( b2TaskCallback* task, void* taskContext, void* userContext );
void b2SchedulerFinishTask( void* userTask, void* userContext );
