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
        .addClass('mdl-data-table-diagnostic')
        .addClass('mdl-js-data-table')
        .appendTo(this.viewer);

        this.dataTables[hardware_id].thead = $('<thead></thead>')
          .appendTo(this.dataTables[hardware_id]);
        this.dataTables[hardware_id].headRow = $('<tr></tr>').appendTo(this.dataTables[hardware_id].thead);
        this.dataTables[hardware_id].icon = $('<td><i class="material-icons">chevron_right</i></td>').appendTo(this.dataTables[hardware_id].headRow);
        this.dataTables[hardware_id].hardwareIdDisplay = $('<td><b>' + hardware_id + '</b></td>').appendTo(this.dataTables[hardware_id].headRow);
        this.dataTables[hardware_id].messageDisplay = $('<td></td>').appendTo(this.dataTables[hardware_id].headRow);
        this.dataTables[hardware_id].tbody = $("<tbody></tbody>").appendTo(this.dataTables[hardware_id]).hide();

        let that = this;
        this.dataTables[hardware_id].thead[0].addEventListener('click', function(e) {
          that.dataTables[hardware_id].tbody.toggle();
          if(that.dataTables[hardware_id].icon.children('i').text() === "chevron_right") {
            that.dataTables[hardware_id].icon.children('i').text("expand_more");
          } else {
            that.dataTables[hardware_id].icon.children('i').text("chevron_right");
          }
        })
      }

      let status = this.diagnosticStatuses[hardware_id];

      this.dataTables[hardware_id].messageDisplay.text(status._message);

      if(status._level === 0) {
        this.dataTables[hardware_id].headRow.addClass("diagnostic-level-ok");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-warn");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-error");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-stale");
      } else if(status._level === 1) {
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-ok");
        this.dataTables[hardware_id].headRow.addClass("diagnostic-level-warn");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-error");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-stale");
      } else if(status._level === 2) {
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-ok");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-warn");
        this.dataTables[hardware_id].headRow.addClass("diagnostic-level-error");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-stale");
      } else if(status._level === 3) {
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-ok");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-warn");
        this.dataTables[hardware_id].headRow.removeClass("diagnostic-level-error");
        this.dataTables[hardware_id].headRow.addClass("diagnostic-level-stale");
      }

      this.dataTables[hardware_id].tbody.empty();
      let html = "";
      for(let key in status) {
        if(key[0] === "_") continue;
        html += '<tr><td>&nbsp;</td><td>' + key + '</td><td>' + status[key] + "</td></tr>";
      }
      $(html).appendTo(this.dataTables[hardware_id].tbody);
    }
  }
}

DiagnosticViewer.friendlyName = "Aggregated diagnostics";

DiagnosticViewer.supportedTypes = [
    "diagnostic_msgs/msg/DiagnosticArray",
];

Viewer.registerViewer(DiagnosticViewer);