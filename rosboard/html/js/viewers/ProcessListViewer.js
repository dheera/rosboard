"use strict";

// Viewer for _top (process list, non-ros)

class ProcessListViewer extends Viewer {
    /**
      * Gets called when Viewer is first initialized.
      * @override
    **/
    onCreate() {
        this.card.title.text("ProcessListViewer");

        // wrapper and wrapper2 are css BS that are necessary to 
        // have something that is 100% width but fixed aspect ratio
        this.wrapper = $('<div></div>')
            .css({
                "position": "relative",
                "width": "100%",
            })
            .appendTo(this.card.content);
        
        this.wrapper2 = $('<div></div>')
            .css({
                "width": "100%",
                "padding-bottom": "80%",
                "background": "#101010",
                "position": "relative",
                "overflow-x": "hidden",
                "overflow-y": "scroll",
            })
            .appendTo(this.wrapper);

        // actual log container, put it inside wrapper2
        this.processTable = $('<table></table>')
            .addClass("monospace")
            .css({
                "position": "absolute",
                "width": "100%",
                "height": "100%",
                "font-size": "7pt",
                "line-height": "1.4em",
                "overflow-y": "hidden",
                "overflow-x": "hidden",
            })
            .appendTo(this.wrapper2);

        super.onCreate();
    }

    onData(msg) {
        this.card.title.text(msg._topic_name);
        
        this.processTable.empty();
        
        $('<tr></tr>')
                .append($("<th>PID</th>").css({"text-align": "left"}))
                .append($("<th>CPU</th>").css({"text-align": "left"}))
                .append($("<th>MEM</th>").css({"text-align": "left"}))
                .append($("<th>USER</th>").css({"text-align": "left"}))
                .append($("<th>COMMAND</th>").css({"text-align": "left"}))
                .appendTo(this.processTable);

        for(let i in msg.processes) {
            let process = msg.processes[i];
            $('<tr></tr>')
                .append($("<td>" + process.pid + "</td>"))
                .append($("<td>" + process.cpu + "</td>"))
                .append($("<td>" + process.mem + "</td>"))
                .append($("<td>" + process.user + "</td>"))
                .append($("<td>" + process.command + "</td>"))
                .appendTo(this.processTable);
        }
    }
}

ProcessListViewer.supportedTypes = [
    "rosboard_msgs/msg/ProcessList",
];

Viewer.registerViewer(ProcessListViewer);
