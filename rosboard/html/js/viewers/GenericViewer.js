"use strict";

// GenericViewer just displays message fields and values in a table.
// It can be used on any ROS type.

class GenericViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewerNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    this.viewerNodeFadeTimeout = null;

    this.expandFields = { };
    this.fieldNodes = { };
    this.dataTable = $('<table></table>')
          .addClass('mdl-data-table')
          .addClass('mdl-js-data-table')
          .css({'width': '100%', 'min-height': '30pt', 'table-layout': 'fixed' })
          .appendTo(this.viewerNode);

    super.onCreate();
  }

  onData(data) {
      this.card.title.text(data._topic_name);

      for(let field in data) {
          if(field[0] === "_") continue;
          // if(field === "header") continue;
          // if(field === "name") continue;

          if(!this.fieldNodes[field]) {
              let tr = $('<tr></tr>')
                .appendTo(this.dataTable);
              $('<td></td>')
                .addClass('mdl-data-table__cell--non-numeric')
                .text(field)
                .css({'width': '40%', 'font-weight': 'bold', 'overflow': 'hidden', 'text-overflow': 'ellipsis'})
                .appendTo(tr);
              this.fieldNodes[field] = $('<td></td>')
                .addClass('mdl-data-table__cell--non-numeric')
                .addClass('monospace')
                .css({'overflow': 'hidden', 'text-overflow': 'ellipsis'})
                .appendTo(tr);
              let that = this;
              this.fieldNodes[field].click(() => {that.expandFields[field] = !that.expandFields[field]; });
          }

        if(data[field].uuid) {
            this.fieldNodes[field].text(data[field].uuid.map((byte) => ((byte<16) ? "0": "") + (byte & 0xFF).toString(16)).join(''));
            this.fieldNodes[field].css({"color": "#808080"});
            continue;
        }
        
        if(typeof(data[field])==="boolean") {
          if(data[field] === true) {
              this.fieldNodes[field].text("true");
              this.fieldNodes[field].css({"color": "#80ff80"});
          } else {
              this.fieldNodes[field].text("false");
              this.fieldNodes[field].css({"color": "#ff8080"});
          }
          continue;
        }

        if(data.__comp && data.__comp.includes(field)) {
          this.fieldNodes[field][0].innerHTML = "(compressed)";
          continue;
        }

        if(this.expandFields[field]) {
          this.fieldNodes[field][0].innerHTML = (
            JSON.stringify(data[field], null, '  ')
              .replace(/\n/g, "<br>")
              .replace(/ /g, "&nbsp;")
          );
        } else {
          this.fieldNodes[field][0].innerHTML = JSON.stringify(data[field], null, '  ');
        }
      }
  }
}

GenericViewer.friendlyName = "Raw data";

GenericViewer.supportedTypes = [
    "*",
];

Viewer.registerViewer(GenericViewer);