/*
 *    Copyright 2017 Rethink Robotics
 *
 *    Copyright 2017 Chris Smith
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

'use strict';

var _require = require('./ClientStates.js');

const CommState = _require.CommState;


const EventEmitter = require('events');
const msgUtils = require('../utils/message_utils.js');

const log = require('../lib/Logging.js').getLogger('actionlib_nodejs');
let GoalStatuses = null;

class ClientGoalHandle extends EventEmitter {
  constructor(actionGoal, actionClientInterface) {
    super();

    this._goal = actionGoal;
    this._clientInterface = actionClientInterface;

    if (GoalStatuses === null) {
      GoalStatuses = msgUtils.requireMsgPackage('actionlib_msgs').msg.GoalStatus.Constants;
    }

    this._state = CommState.WAITING_FOR_GOAL_ACK;

    this._goalStatus = null;
    this._result = null;
    this._active = true;
  }

  reset() {
    this._active = false;
    this._actionClient = null;
    this.removeAllListeners();
  }

  getGoalStatus() {
    return this._goalStatus;
  }

  resend() {
    if (!this._active) {
      log.error('Trying to resend on an inactive ClientGoalHandle!');
    }

    this._actionClient.sendGoal(this._goal);
  }

  cancel() {
    if (!this._active) {
      log.error('Trying to cancel on an inactive ClientGoalHandle!');
    }

    switch (this._state) {
      case CommState.WAITING_FOR_GOAL_ACK:
      case CommState.PENDING:
      case CommState.ACTIVE:
      case CommState.WAITING_FOR_CANCEL_ACK:
        break;
      case CommState.WAITING_FOR_RESULT:
      case CommState.RECALLING:
      case CommState.PREEMPTING:
      case CommState.DONE:
        log.debug('Got a cancel request while in state [%s], ignoring it', this._state);
        return;
      default:
        log.error('BUG: Unhandled CommState: %s', this._state);
        return;
    }

    this._actionClient.cancel(this._goal.goal_id.id, { secs: 0, nsecs: 0 });
    this._transition(CommState.WAITING_FOR_CANCEL_ACK);
  }

  getResult() {
    if (!this._active) {
      log.error('Trying to getResult on an inactive ClientGoalHandle!');
    }

    if (this._result) {
      return this._result.result;
    }
  }

  getTerminalState() {
    if (!this._active) {
      log.error('Trying to getTerminalState on an inactive ClientGoalHandle!');
    }

    if (this._state !== CommState.DONE) {
      log.warn('Asking for terminal state when we\'re in %s', this._state);
    }

    if (this._goalStatus) {
      switch (this._goalStatus.status) {
        case GoalStatuses.PENDING:
        case GoalStatuses.ACTIVE:
        case GoalStatuses.PREEMPTING:
        case GoalStatuses.RECALLING:
          log.error('Asking for terminal state, but latest goal status is %s', this._goalStatus.status);
          return GoalStatuses.LOST;
        case GoalStatuses.PREEMPTED:
        case GoalStatuses.SUCCEEDED:
        case GoalStatuses.ABORTED:
        case GoalStatuses.REJECTED:
        case GoalStatuses.RECALLED:
        case GoalStatuses.LOST:
          return this._goalStatus.status;
        default:
          log.error('Unknown goal status: %s'.this._goalStatus.status);
      }
    }
  }

  getCommState() {
    return this._state;
  }

  isExpired() {
    return !this._active;
  }

  updateFeedback(feedback) {
    this.emit('feedback', feedback);
  }

  updateResult(actionResult) {
    this._goalStatus = actionResult.status;
    this._result = actionResult;

    switch (this._state) {
      case CommState.WAITING_FOR_GOAL_ACK:
      case CommState.PENDING:
      case CommState.ACTIVE:
      case CommState.WAITING_FOR_RESULT:
      case CommState.WAITING_FOR_CANCEL_ACK:
      case CommState.RECALLING:
      case CommState.PREEMPTING:
        // trigger all the state transitions users would expect
        this.updateStatus(actionResult.status);

        this._transition(CommState.DONE);
        break;
      case CommState.DONE:
        log.error('Got a result when we were already in the DONE state');
        break;
      default:
        log.error('In a funny comm state: %s', this._state);
    }
  }

  updateStatus(status) {
    // it's apparently possible to receive old GoalStatus messages, even after
    // transitioning to a terminal state.
    if (this._state === CommState.DONE) {
      return;
    }
    // else
    if (status) {
      this._goalStatus = status;
    } else {
      // this goal wasn't included in the latest status message!
      // it may have been lost
      if (this._state !== CommState.WAITING_FOR_GOAL_ACK && this._state !== CommState.WAITING_FOR_RESULT && this._state !== CommState.DONE) {
        log.warn('Transitioning goal to LOST');
        this._goalStatus.status === GoalStatuses.LOST;
        this._transition(CommState.DONE);
      }
      return;
    }

    switch (this._state) {
      case CommState.WAITING_FOR_GOAL_ACK:
        switch (status.status) {
          case GoalStatuses.PENDING:
            this._transition(CommState.PENDING);
            break;
          case GoalStatuses.ACTIVE:
            this._transition(CommState.ACTIVE);
            break;
          case GoalStatuses.PREEMPTED:
            this._transition(CommState.ACTIVE);
            this._transition(CommState.PREEMPTING);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.SUCCEEDED:
            this._transition(CommState.ACTIVE);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.ABORTED:
            this._transition(CommState.ACTIVE);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.REJECTED:
            this._transition(CommState.PENDING);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.RECALLED:
            this._transition(CommState.PENDING);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.PREEMPTING:
            this._transition(CommState.ACTIVE);
            this._transition(CommState.PREEMPTING);
            break;
          case GoalStatuses.RECALLING:
            this._transition(CommState.PENDING);
            this._transition(CommState.RECALLING);
            break;
          default:
            log.error('BUG: Got an unknown status from the ActionServer: status = ' + status.status);
            break;
        }
        break;
      case CommState.PENDING:
        switch (status.status) {
          case GoalStatuses.PENDING:
            break;
          case GoalStatuses.ACTIVE:
            this._transition(CommState.ACTIVE);
            break;
          case GoalStatuses.PREEMPTED:
            this._transition(CommState.ACTIVE);
            this._transition(CommState.PREEMPTING);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.SUCCEEDED:
            this._transition(CommState.ACTIVE);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.ABORTED:
            this._transition(CommState.ACTIVE);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.REJECTED:
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.RECALLED:
            this._transition(CommState.RECALLING);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.PREEMPTING:
            this._transition(CommState.ACTIVE);
            this._transition(CommState.PREEMPTING);
            break;
          case GoalStatuses.RECALLING:
            this._transition(CommState.RECALLING);
            break;
          default:
            log.error('BUG: Got an unknown status from the ActionServer: status = ' + status.status);
            break;
        }
        break;
      case CommState.ACTIVE:
        switch (status.status) {
          case GoalStatuses.PENDING:
            log.error('Invalid transition from ACTIVE to PENDING');
            break;
          case GoalStatuses.REJECTED:
            log.error('Invalid transition from ACTIVE to REJECTED');
            break;
          case GoalStatuses.RECALLED:
            log.error('Invalid transition from ACTIVE to RECALLED');
            break;
          case GoalStatuses.RECALLING:
            log.error('Invalid transition from ACTIVE to RECALLING');
            break;
          case GoalStatuses.ACTIVE:
            break;
          case GoalStatuses.PREEMPTED:
            this._transition(CommState.PREEMPTING);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.SUCCEEDED:
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.ABORTED:
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.PREEMPTING:
            this._transition(CommState.PREEMPTING);
            break;
          default:
            log.error('BUG: Got an unknown status from the ActionServer: status = ' + status.status);
            break;
        }
        break;
      case CommState.WAITING_FOR_RESULT:
        switch (status.status) {
          case GoalStatuses.PENDING:
            log.error('Invalid transition from WAITING_FOR_RESULT to PENDING');
            break;
          case GoalStatuses.PREEMPTING:
            log.error('Invalid transition from WAITING_FOR_RESULT to PREEMPTING');
            break;
          case GoalStatuses.RECALLING:
            log.error('Invalid transition from WAITING_FOR_RESULT to RECALLING');
            break;
          case GoalStatuses.ACTIVE:
          case GoalStatuses.PREEMPTED:
          case GoalStatuses.SUCCEEDED:
          case GoalStatuses.ABORTED:
          case GoalStatuses.REJECTED:
          case GoalStatuses.RECALLED:
            break;
          default:
            log.error('BUG: Got an unknown status from the ActionServer: status = ' + status.status);
            break;
        }
        break;
      case CommState.WAITING_FOR_CANCEL_ACK:
        switch (status.status) {
          case GoalStatuses.PENDING:
          case GoalStatuses.ACTIVE:
            break;
          case GoalStatuses.PREEMPTED:
          case GoalStatuses.SUCCEEDED:
          case GoalStatuses.ABORTED:
            this._transition(CommState.PREEMPTING);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.RECALLED:
            this._transition(CommState.RECALLING);
            this._transition(CommState.WAITING_FOR_RESULT);
          case GoalStatuses.REJECTED:
            this._transition(CommState.WAITING_FOR_RESULT);
          case GoalStatuses.PREEMPTING:
            this._transition(CommState.PREEMPTING);
            break;
          case GoalStatuses.RECALLING:
            this._transition(CommState.RECALLING);
            break;
          default:
            log.error('BUG: Got an unknown status from the ActionServer: status = ' + status.status);
            break;
        }
        break;
      case CommState.RECALLING:
        switch (status.status) {
          case GoalStatuses.PENDING:
            log.error('Invalid transition from RECALLING to PENDING');
            break;
          case GoalStatuses.ACTIVE:
            log.error('Invalid transition from RECALLING to ACTIVE');
            break;
          case GoalStatuses.PREEMPTED:
          case GoalStatuses.SUCCEEDED:
          case GoalStatuses.ABORTED:
            this._transition(CommState.PREEMPTING);
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.RECALLED:
            this._transition(CommState.WAITING_FOR_RESULT);
          case GoalStatuses.REJECTED:
            this._transition(CommState.WAITING_FOR_RESULT);
          case GoalStatuses.PREEMPTING:
            this._transition(CommState.PREEMPTING);
            break;
          case GoalStatuses.RECALLING:
            break;
          default:
            log.error('BUG: Got an unknown status from the ActionServer: status = ' + status.status);
            break;
        }
        break;
      case CommState.PREEMPTING:
        switch (status.status) {
          case GoalStatuses.PENDING:
            log.error('Invalid transition from PREEMPTING to PENDING');
            break;
          case GoalStatuses.ACTIVE:
            log.error('Invalid transition from PREEMPTING to ACTIVE');
            break;
          case GoalStatuses.REJECTED:
            log.error('Invalid transition from PREEMPTING to REJECTED');
          case GoalStatuses.RECALLING:
            log.error('Invalid transition from PREEMPTING to RECALLING');
            break;
          case GoalStatuses.RECALLED:
            log.error('Invalid transition from PREEMPTING to RECALLED');
            break;
          case GoalStatuses.PREEMPTED:
          case GoalStatuses.SUCCEEDED:
          case GoalStatuses.ABORTED:
            this._transition(CommState.WAITING_FOR_RESULT);
            break;
          case GoalStatuses.PREEMPTING:
            break;
          default:
            log.error('BUG: Got an unknown status from the ActionServer: status = ' + status.status);
            break;
        }
        break;
      case CommState.DONE:
        // I'm pretty sure we can never get here... but actionlib has it so I'm going to
        // follow suit.
        switch (status.status) {
          case GoalStatuses.PENDING:
            log.error('Invalid transition from DONE to PENDING');
            break;
          case GoalStatuses.ACTIVE:
            log.error('Invalid transition from DONE to ACTIVE');
            break;
          case GoalStatuses.RECALLING:
            log.error('Invalid transition from DONE to RECALLING');
            break;
          case GoalStatuses.PREEMPTING:
            log.error('Invalid transition from DONE to PREEMPTING');
            break;
          case GoalStatuses.RECALLED:
          case GoalStatuses.REJECTED:
          case GoalStatuses.PREEMPTED:
          case GoalStatuses.SUCCEEDED:
          case GoalStatuses.ABORTED:
            break;
          default:
            log.error('BUG: Got an unknown status from the ActionServer: status = ' + status.status);
            break;
        }
        break;
      default:
        log.error('In a funny comm state: %s', this._state);
    }
  }

  _transition(newState) {
    log.debug('Trying to transition to %s', newState);
    this._state = newState;
    this.emit('transition');
  }
}

module.exports = ClientGoalHandle;