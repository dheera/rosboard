"use strict";

class DiagnosticViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewer = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    this.diagnosticStatuses = {};

    this.expandFields = { };
    this.fieldNodes = { };
    this.dataTables = { };
    /*this.dataTable = $('<table></table>')
          .addClass('mdl-data-table')
          .addClass('mdl-js-data-table')
          .css({'width': '100%', 'min-height': '30pt', 'table-layout': 'fixed' })
          .appendTo(this.viewer);*/

    super.onCreate();
  }

  onData(msg) {
    this.card.title.text(msg._topic_name);

    for(let i in msg.status) {
      let status = {};
      status["_level"] = msg.status[i].level;
      status["_name"] = msg.status[i].name;
      status["_message"] = msg.status[i].message;
      status["_hardware_id"] = msg.status[i].hardware_id;
      for(let j in msg.status[i].values) {
        status[msg.status[i].values[j].key] = msg.status[i].values[j].value;
      }
      this.diagnosticStatuses[status._hardware_id] = status;
    }

    for(let hardware_id in this.diagnosticStatuses) {
      if(!this.dataTables[hardware_id]) {
        this.dataTables[hardware_id] = $('<table></table>')
        .addClass('mdl-data-table')
        .addClass('mdl-data-table-compact')
        .addClass('mdl-js-data-table')
        .css({'width': '100%', 'min-height': '30pt', 'table-layout': 'fixed' })
        .appendTo(this.viewer);
      }
      let status = this.diagnosticStatuses[hardware_id];
      this.dataTables[hardware_id].empty();
      this.dataTables[hardware_id].append($('<tr style="background:#505050;"><td style="width:30%"><b>' + hardware_id + '</b></td><td>' + status._message + "</td></tr>"));
      for(let key in status) {
        if(key[0] === "_") continue;
        this.dataTables[hardware_id].append($('<tr><td style="width:30%">' + key + '</td><td>' + status[key] + "</td></tr>"));
      }
    }
  }
}

DiagnosticViewer.supportedTypes = [
    "diagnostic_msgs/msg/DiagnosticArray",
];

Viewer.registerViewer(DiagnosticViewer);