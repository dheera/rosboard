class LogViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.BGCOLORS = ['#c0c0c0', '#808080', '#cc8000', '#cc0000', '#ff0000'];
    this.FGCOLORS = ['#ffffff', '#ffffff', '#ffffff', '#ffffff', '#ff8000'];
    this.listeners = {};
    this.query = '';
    this.optionsBox = $(
      '<div style="padding-left:15pt;padding-right:15pt;height:calc( 50pt );overflow:hidden;">' + 
      '<div class="mdl-textfield mdl-js-textfield">' +
      '<input class="mdl-textfield__input" type="text" id="sample1">' +
      '<label class="mdl-textfield__label" for="sample1">Search ...</label>' +
      '</div>' +
      '</div>')
      .appendTo(this.cardContentNode);
    this.logBoxNode = $('<div style="padding:0;width:100%;height:calc( 100% - 50pt );overflow:hidden;"></div>')
      .appendTo(this.cardContentNode);

    var that = this;
    this.optionsBox.find('input').keyup(function(e) {
      that.setQuery($(this).val());
    });

    super.onCreate();
  }

  /**
    * Gets called when Viewer is about to be destroyed.
    * @override
  **/
  onDestroy() {
    for(var topic in this.listeners) {
      this.removeTopic(topic);
    }
    super.onDestroy();
  }

  /**
    * Adds a topic to the viewer.
    * @override
  **/
  addTopic(topic) {
    if(topic in this.listeners) return;
    var listener = new ROSLIB.Topic({
      ros : ros,
      name : topic,
      messageType : 'rosgraph_msgs/Log'
    });
    this.listeners[topic] = listener;
    var that = this;
    listener.subscribe(function(message) {
      var realLevel = parseInt(Math.log2(message.level))
      var visibility = that.query === '' || (message.name + ' ' + message.msg).indexOf(that.query) !== -1;
      $('<div></div>').text(message.msg).prepend('<b>' + message.name + '</b> ').css({
          'background': that.BGCOLORS[realLevel],
          'color': that.FGCOLORS[realLevel],
          'padding': '5pt',
          'margin-top': '3pt',
          'margin-bottom': '3pt',
          'margin-left': '15pt',
          'margin-right': '15pt',
          'font-size': '10pt',
          'display': (visibility ? '':'none'),
          'font-family': 'Titillium Web,sans-serif',
      }).appendTo(that.logBoxNode);
      while(that.logBoxNode.children().length > 512) {
        that.logBoxNode.children().first().hide().remove();
      }
      that.logBoxNode[0].scrollTop = that.logBoxNode[0].scrollHeight;
    });
  }

  /**
    * Removes a topic from the viewer.
    * @override
  **/
  removeTopic(topic) {
    if(!(topic in this.listeners)) return;
    this.listeners[topic].unsubscribe();
    delete(this.listeners[topic]);
  }

  /**
    * Returns a list of topics subscribed to.
    * @override
  **/
  getTopics() {
    return Object.keys(this.listeners);
  }

  /**
    * Sets a user-defined filter for log messages to display.
  **/
  setQuery(query) {
    this.query = query;
    var that = this;
    this.logBoxNode.children().each(function(i, e) {
      visibility = that.query === '' || ($(e).text()).indexOf(that.query) !== -1;
      $(e).css('display', (visibility ? '' : 'none'));
    });
  }
}

LogViewer._TYPE = 'rosgraph_msgs/Log';
addViewer(LogViewer);

