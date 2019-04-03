'use strict';

const rosnodejs = require('../index.js');

var _rosnodejs$require$ms = rosnodejs.require('actionlib_tutorials').msg;

const FibonacciGoal = _rosnodejs$require$ms.FibonacciGoal,
      FibonacciResult = _rosnodejs$require$ms.FibonacciResult,
      FibonacciFeedback = _rosnodejs$require$ms.FibonacciFeedback;


rosnodejs.initNode('fibonacci').then(() => {
  function executeCallback(goal) {
    const feedback = new FibonacciFeedback();

    rosnodejs.log.info('got goal!');
    feedback.sequence.push(0, 1);
    rosnodejs.log.info('Executing fibonacci sequence %d', goal.order);

    function _exec(iter, done) {
      if (iter <= goal.order) {
        if (as.isPreemptRequested() || !rosnodejs.ok()) {
          rosnodejs.log.warn('PREEMPTED!');
          as.setPreempted();
          done();
        } else {
          feedback.sequence.push(feedback.sequence[iter] + feedback.sequence[iter - 1]);
          rosnodejs.log.info('Update: %j', feedback.sequence);
          as.publishFeedback(feedback);
          setTimeout(_exec, 100, iter + 1, done);
        }
      } else {
        // done!
        const result = new FibonacciResult();
        result.sequence = feedback.sequence;
        rosnodejs.log.info('Succeeded!');
        as.setSucceeded(result);
        done();
      }
    }

    return new Promise(function (resolve) {
      _exec(1, resolve);
    });
  }

  const as = new rosnodejs.SimpleActionServer({
    nh: rosnodejs.nh,
    type: 'actionlib_tutorials/Fibonacci',
    actionServer: '/fibonacci',
    executeCallback
  });

  as.start();

  const ac = new rosnodejs.SimpleActionClient({
    nh: rosnodejs.nh,
    type: 'actionlib_tutorials/Fibonacci',
    actionServer: '/fibonacci'
  });

  ac.waitForServer().then(() => {
    rosnodejs.log.info('Connected to action server!');

    const goal = { order: 20 };
    ac.sendGoal(goal, function (result) {
      rosnodejs.log.info('Result: %j', result);
    }, function () {
      console.log('ACTIVE');
    }, function (feedback) {
      rosnodejs.log.info('%j', feedback);
    });

    ac.waitForResult({ secs: 30, nsecs: 0 }).then(finished => {
      if (finished) {
        rosnodejs.log.info('Action finished: %s', ac.getState());
      } else {
        rosnodejs.log.info('Action did not finish before the time out');
      }

      rosnodejs.shutdown().then(() => {
        rosnodejs.log.info('Shutdown complete!');
        rosnodejs.reset();
      });
    });
  });
}).catch(function (err) {
  console.error(err.stack);
});