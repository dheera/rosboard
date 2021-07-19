"use strict";

// Viewer for /rosout and other logs that can be expressed in
// rcl_interfaces/msgs/Log format.

class LogViewer extends Viewer {
    /**
      * Gets called when Viewer is first initialized.
      * @override
    **/
    onCreate() {
        this.card.title.text("LogViewer");

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
                "overflow": "hidden",
            })
            .appendTo(this.wrapper);

        // actual log container, put it inside wrapper2
        this.logContainer = $('<div></div>')
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

        // add the first log
        $('<div></div>')
            .text("Logs will appear here.")
            .appendTo(this.logContainer);   

        super.onCreate();

        let that = this;
        this.logScrollTimeout = setTimeout(() => {
            that.logContainer[0].scrollTop = that.logContainer[0].scrollHeight;
        }, 1000);
    }

    onData(msg) {
        while(this.logContainer.children().length > 30) {
            this.logContainer.children()[0].remove();
        }

        this.card.title.text(msg._topic_name);

        let color = "#c0c0c0"; // default color
        let level_text = "";

        // set colors based on log level, if defined
        // 10-20-30-40-50 is ROS2, 1-2-4-8-16 is ROS1
        if(msg.level === 10 || msg.level === 1) { level_text = "DEBUG"; color = "#00a000"; }
        if(msg.level === 20 || msg.level === 2) { level_text = "INFO"; color = "#a0a0a0"; }
        if(msg.level === 30 || msg.level === 4) { level_text = "WARN"; color = "#c0c000"; }
        if(msg.level === 40 || msg.level === 8) { level_text = "ERROR"; color = "#ff4040"; }
        if(msg.level === 50 || msg.level === 16) { level_text = "FATAL"; color = "#ff0000"; }

        let text = "";
        if(level_text !== "") text += "[" + level_text + "] "
        if(msg.name) text += "[" + msg.name + "] ";
        text += msg.msg;

        text = text
            .replace(/&/g, "&amp;")
            .replace(/</g, "&lt;")
            .replace(/>/g, "&gt;")
            .replace(/"/g, "&quot;")
            .replace(/'/g, "&#039;")
            .replace(/\n/g, "<br>\n")
            .replace(/(\[[0-9\.]*\])/g, '<span style="color:#008000;">$1</span>');

        $('<div></div>')
            .html(text)
            .css({ "color": color })
            .appendTo(this.logContainer);

        this.logContainer[0].scrollTop = this.logContainer[0].scrollHeight;
    }
}

LogViewer.friendlyName = "Log view";

LogViewer.supportedTypes = [
    "rcl_interfaces/msg/Log",
    "rosgraph_msgs/msg/Log",
];

Viewer.registerViewer(LogViewer);