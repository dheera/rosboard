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

const timeUtils = require('../utils/time_utils.js');
const msgUtils = require('../utils/message_utils.js');
const EventEmitter = require('events');

let GoalID = null;
let goalCount = 0;

class ActionServerInterface extends EventEmitter {
  constructor(options) {
    super();

    if (GoalID === null) {
      GoalID = msgUtils.requireMsgPackage('actionlib_msgs').msg.GoalID;
    }

    this._actionType = options.type;

    this._actionServer = options.actionServer;

    const nh = options.nh;
    this._nh = nh;

    const goalOptions = Object.assign({ queueSize: 50 }, options.goal);
    this._goalSub = nh.subscribe(this._actionServer + '/goal', this._actionType + 'ActionGoal', msg => {
      this._handleGoal(msg);
    }, goalOptions);

    const cancelOptions = Object.assign({ queueSize: 50 }, options.cancel);
    this._cancelSub = nh.subscribe(this._actionServer + '/cancel', 'actionlib_msgs/GoalID', msg => {
      this._handleCancel(msg);
    }, cancelOptions);

    const statusOptions = Object.assign({ queueSize: 50 }, options.status);
    this._statusPub = nh.advertise(this._actionServer + '/status', 'actionlib_msgs/GoalStatusArray', statusOptions);

    const feedbackOptions = Object.assign({ queueSize: 50 }, options.feedback);
    this._feedbackPub = nh.advertise(this._actionServer + '/feedback', this._actionType + 'ActionFeedback', feedbackOptions);

    const resultOptions = Object.assign({ queueSize: 50 }, options.result);
    this._resultPub = nh.advertise(this._actionServer + '/result', this._actionType + 'ActionResult', resultOptions);
  }

  getType() {
    return this._actionType;
  }

  generateGoalId() {
    const now = timeUtils.now();
    return new GoalID({
      id: `${this._nh.getNodeName()}-${goalCount++}-${now.sec}.${now.nsec}`,
      stamp: now
    });
  }

  shutdown() {
    return Promise.all([this._goalSub.shutdown(), this._cancelSub.shutdown(), this._statusPub.shutdown(), this._feedbackPub.shutdown(), this._resultPub.shutdown()]);
  }

  _handleGoal(msg) {
    this.emit('goal', msg);
  }

  _handleCancel(msg) {
    this.emit('cancel', msg);
  }

  publishResult(resultMsg) {
    this._resultPub.publish(resultMsg);
  }

  publishFeedback(feedbackMsg) {
    this._feedbackPub.publish(feedbackMsg);
  }

  publishStatus(statusMsg) {
    this._statusPub.publish(statusMsg);
  }
}

module.exports = ActionServerInterface;