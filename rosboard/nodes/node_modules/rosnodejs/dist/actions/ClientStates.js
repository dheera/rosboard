'use strict';

const SimpleGoalState = {
  PENDING: 'PENDING',
  ACTIVE: 'ACTIVE',
  DONE: 'DONE'
};

const SimpleClientGoalState = {
  PENDING: 'PENDING',
  ACTIVE: 'ACTIVE',
  RECALLED: 'RECALLED',
  REJECTED: 'REJECTED',
  PREEMPTED: 'PREEMPTED',
  ABORTED: 'ABORTED',
  SUCCEEDED: 'SUCCEEDED',
  LOST: 'LOST'
};

const CommState = {
  WAITING_FOR_GOAL_ACK: 0,
  PENDING: 1,
  ACTIVE: 2,
  WAITING_FOR_RESULT: 3,
  WAITING_FOR_CANCEL_ACK: 4,
  RECALLING: 5,
  PREEMPTING: 6,
  DONE: 7
};

module.exports = {
  CommState,
  SimpleGoalState,
  SimpleClientGoalState
};