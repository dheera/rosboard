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
      let idx_name = msg.status[i].name.split("/")[1];
      let status = {}
      if (this.diagnosticStatuses.hasOwnProperty(idx_name)) {
        status = this.diagnosticStatuses[idx_name];
        if (msg.status[i].level > status["_level"]) {
          status["_level"] = msg.status[i].level;
          status["_message"] = msg.status[i].message;
        }
      } else {
        status["_level"] = msg.status[i].level;
        status["_name"] = idx_name;
        status["_message"] = msg.status[i].message;
        status["_hardware_id"] = msg.status[i].hardware_id;
      }
      for(let j in msg.status[i].values) {
        status[msg.status[i].values[j].key] = msg.status[i].values[j].value;
      }
      this.diagnosticStatuses[status._name] = status;
    }

    for(let key in this.diagnosticStatuses) {
      if(!this.dataTables[key]) {
        this.dataTables[key] = $('<table></table>')
        .addClass('mdl-data-table')
        .addClass('mdl-data-table-diagnostic')
        .addClass('mdl-js-data-table')
        .appendTo(this.viewer);

        this.dataTables[key].thead = $('<thead></thead>')
          .appendTo(this.dataTables[key]);
        this.dataTables[key].headRow = $('<tr></tr>').appendTo(this.dataTables[key].thead);
        this.dataTables[key].icon = $('<td><i class="material-icons">chevron_right</i></td>').appendTo(this.dataTables[key].headRow);
        this.dataTables[key].hardwareIdDisplay = $('<td><b>' + key + '</b></td>').appendTo(this.dataTables[key].headRow);
        this.dataTables[key].messageDisplay = $('<td></td>').appendTo(this.dataTables[key].headRow);
        this.dataTables[key].tbody = $("<tbody></tbody>").appendTo(this.dataTables[key]).hide();

        let that = this;
        this.dataTables[key].thead[0].addEventListener('click', function(e) {
          that.dataTables[key].tbody.toggle();
          if(that.dataTables[key].icon.children('i').text() === "chevron_right") {
            that.dataTables[key].icon.children('i').text("expand_more");
          } else {
            that.dataTables[key].icon.children('i').text("chevron_right");
          }
        })
      }

      let status = this.diagnosticStatuses[key];

      this.dataTables[key].messageDisplay.text(status._message);

      if(status._level === 0) {
        this.dataTables[key].headRow.addClass("diagnostic-level-ok");
        this.dataTables[key].headRow.removeClass("diagnostic-level-warn");
        this.dataTables[key].headRow.removeClass("diagnostic-level-error");
        this.dataTables[key].headRow.removeClass("diagnostic-level-stale");
      } else if(status._level === 1) {
        this.dataTables[key].headRow.removeClass("diagnostic-level-ok");
        this.dataTables[key].headRow.addClass("diagnostic-level-warn");
        this.dataTables[key].headRow.removeClass("diagnostic-level-error");
        this.dataTables[key].headRow.removeClass("diagnostic-level-stale");
      } else if(status._level === 2) {
        this.dataTables[key].headRow.removeClass("diagnostic-level-ok");
        this.dataTables[key].headRow.removeClass("diagnostic-level-warn");
        this.dataTables[key].headRow.addClass("diagnostic-level-error");
        this.dataTables[key].headRow.removeClass("diagnostic-level-stale");
      } else if(status._level === 3) {
        this.dataTables[key].headRow.removeClass("diagnostic-level-ok");
        this.dataTables[key].headRow.removeClass("diagnostic-level-warn");
        this.dataTables[key].headRow.removeClass("diagnostic-level-error");
        this.dataTables[key].headRow.addClass("diagnostic-level-stale");
      }

      this.dataTables[key].tbody.empty();
      let html = "";
      for(let key in status) {
        if(key[0] === "_") continue;
        html += '<tr><td>&nbsp;</td><td>' + key + '</td><td>' + status[key] + "</td></tr>";
      }
      $(html).appendTo(this.dataTables[key].tbody);
    }
  }
}

DiagnosticViewer.friendlyName = "Aggregated diagnostics";

DiagnosticViewer.supportedTypes = [
    "diagnostic_msgs/msg/DiagnosticArray",
];

Viewer.registerViewer(DiagnosticViewer);
