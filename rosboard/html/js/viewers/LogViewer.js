"use strict";

class LogViewer extends Viewer {
    /**
      * Gets called when Viewer is first initialized.
      * @override
    **/
    onCreate() {

        this.card.title.text("ROS");

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

        this.logWindow = $('<div></div>')
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

        $('<div></div>')
            .text("Logs from /rosout will appear here.")
            .appendTo(this.logWindow);   

        super.onCreate();
    }

    onData(msg) {
        while(this.logWindow.children().length > 30) {
            this.logWindow.children()[0].remove();
        }

        this.card.title.text(msg._topic_name);

        let color = "#c0c0c0";
        let level_text = "";
        if(msg.level === 10) { level_text = "DEBUG"; color = "#00a000"; }
        if(msg.level === 20) { level_text = "INFO"; color = "#a0a0a0"; }
        if(msg.level === 30) { level_text = "WARN"; color = "#c0c000"; }
        if(msg.level === 40) { level_text = "ERROR"; color = "#ff4040"; }
        if(msg.level === 50) { level_text = "FATAL"; color = "#ff0000"; }

        let text = "";
        if(level_text !== "") text += "[" + level_text + "] "
        if(msg.name) text += "[" + msg.name + "] ";
        text += msg.msg;

        $('<div></div>')
            .text(text)
            .css({ "color": color })
            .appendTo(this.logWindow);

        this.logWindow[0].scrollTop = this.logWindow[0].scrollHeight;
    }
}

LogViewer.supportedTypes = [
    "rcl_interfaces/msg/Log",
    "rosgraph_msgs/msg/Log",
];

registerViewer(LogViewer);