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

const msgUtils = require('../utils/message_utils.js');
const timeUtils = require('../lib/Time.js');
const log = require('../lib/Logging.js').getLogger('ros.rosnodejs');

let GoalStatus = null;
let GoalStatuses = null;

class GoalHandle {
  /**
   * goalId: An actionlib_msgs/GoalID.
   * actionServer: The ActionServer processing this goal
   * status: A number from actionlib_msgs/GoalStatus, like GoalStatuses.PENDING.
   * goal: The goal message, e.g., a FibonacciGoal. May be left undefined if
   *  this goal is used to represent a cancellation.
   */
  constructor(goalId, actionServer, status, goal) {
    if (goalId.id === '') {
      goalId = actionServer.generateGoalId();
    }

    if (timeUtils.isZeroTime(goalId.stamp)) {
      goalId.stamp = timeUtils.now();
    }

    this.id = goalId.id;

    this._as = actionServer;

    if (GoalStatus === null) {
      GoalStatus = msgUtils.requireMsgPackage('actionlib_msgs').msg.GoalStatus;
      GoalStatuses = GoalStatus.Constants;
    }

    this._status = new GoalStatus({
      status: status || GoalStatuses.PENDING,
      goal_id: goalId
    });

    this._goal = goal;

    this._destructionTime = timeUtils.epoch();
  }

  getGoal() {
    return this._goal;
  }

  getStatusId() {
    return this._status.status;
  }

  getGoalId() {
    return this._status.goal_id;
  }

  getGoalStatus() {
    return this._status;
  }

  publishFeedback(feedback) {
    this._as.publishFeedback(this._status, feedback);
  }

  _setStatus(status, text) {
    this._status.status = status;
    if (text) {
      this._status.text = text;
    }

    // FIXME: just guessing about setting destruction time
    if (this._isTerminalState()) {
      this._destructionTime = timeUtils.now();
    }

    this._as.publishStatus();
  }

  _publishResult(result) {
    this._as.publishResult(this._status, result);
  }

  // For Goal State transitions, See
  // http://wiki.ros.org/actionlib/DetailedDescription#Server_Description

  setCanceled(result) {
    let text = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : '';

    const status = this.getStatusId();
    switch (status) {
      case GoalStatuses.PENDING:
      case GoalStatuses.RECALLING:
        this._setStatus(GoalStatuses.RECALLED, text);
        this._publishResult(result);
        break;
      case GoalStatuses.ACTIVE:
      case GoalStatuses.PREEMPTING:
        this._setStatus(GoalStatuses.PREEMPTED, text);
        this._publishResult(result);
        break;
      default:
        this._logInvalidTransition('setCancelled', status);
        break;
    }
  }

  setCancelled(result) {
    let text = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : '';

    return this.setCanceled(result, text);
  }

  setRejected(result) {
    let text = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : '';

    const status = this.getStatusId();
    switch (status) {
      case GoalStatuses.PENDING:
      case GoalStatuses.RECALLING:
        this._setStatus(GoalStatuses.REJECTED, text);
        this._publishResult(result);
        break;
      default:
        this._logInvalidTransition('setRejected', status);
        break;
    }
  }

  setAccepted() {
    let text = arguments.length > 0 && arguments[0] !== undefined ? arguments[0] : '';

    const status = this.getStatusId();
    switch (status) {
      case GoalStatuses.PENDING:
        this._setStatus(GoalStatuses.ACTIVE, text);
        break;
      case GoalStatuses.RECALLING:
        this._setStatus(GoalStatuses.PREEMPTING, text);
        break;
      default:
        this._logInvalidTransition('setAccepted', status);
        break;
    }
  }

  setAborted(result) {
    let text = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : '';

    const status = this.getStatusId();
    switch (status) {
      case GoalStatuses.PREEMPTING:
      case GoalStatuses.ACTIVE:
        this._setStatus(GoalStatuses.ABORTED, text);
        this._publishResult(result);
        break;
      default:
        this._logInvalidTransition('setAborted', status);
        break;
    }
  }

  setSucceeded(result) {
    let text = arguments.length > 1 && arguments[1] !== undefined ? arguments[1] : '';

    const status = this.getStatusId();
    switch (status) {
      case GoalStatuses.PREEMPTING:
      case GoalStatuses.ACTIVE:
        this._setStatus(GoalStatuses.SUCCEEDED, text);
        this._publishResult(result);
        break;
      default:
        this._logInvalidTransition('setSucceeded', status);
        break;
    }
  }

  setCancelRequested() {
    const status = this.getStatusId();
    switch (status) {
      case GoalStatuses.PENDING:
        this._setStatus(GoalStatuses.RECALLING);
        return true;
      case GoalStatuses.ACTIVE:
        this._setStatus(GoalStatuses.PREEMPTING);
        return true;
      default:
        this._logInvalidTransition('setCancelRequested', status);
        return false;
    }
  }

  _logInvalidTransition(transition, currentStatus) {
    log.warn('Unable to %s from status %s for goal %s', transition, currentStatus, this.id);
  }

  _isTerminalState() {
    return [GoalStatuses.REJECTED, GoalStatuses.RECALLED, GoalStatuses.PREEMPTED, GoalStatuses.ABORTED, GoalStatuses.SUCCEEDED].includes(this._status.status);
  }
}

module.exports = GoalHandle;