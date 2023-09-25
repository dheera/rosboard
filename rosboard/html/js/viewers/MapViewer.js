"use strict";

class MapViewer extends Viewer {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
  onCreate() {
    this.viewer = $('<div></div>')
      .css({'font-size': '11pt'
    , "filter": "invert(100%) saturate(50%)"})
      .appendTo(this.card.content);

    this.mapId = "map-" + Math.floor(Math.random()*10000);

    this.map = $('<div id="' + this.mapId + '"></div>')
      .css({
        "height": "250px",
      })
      .appendTo(this.viewer);

    this.mapLeaflet = L.map(this.mapId).setView([51.505,-0.09], 15);

    this.mapLeaflet.dragging.disable();

    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
    }).addTo(this.mapLeaflet);

    this.marker = null;
    // Initialize an empty array to store marker history
    this.markerHistory = [];
  }

  onData(msg) {
      this.card.title.text(msg._topic_name);
      if(this.marker) this.mapLeaflet.removeLayer(this.marker);

      this.marker = L.marker([msg.latitude, msg.longitude]);
      this.marker.addTo(this.mapLeaflet);
      this.mapLeaflet.setView([msg.latitude, msg.longitude]);
      // Create a new marker and add it to the map
      const newMarker = L.marker([msg.latitude, msg.longitude]);
      newMarker.addTo(this.mapLeaflet);

      // Add the new marker to the history
      this.markerHistory.push(newMarker);

      // Set the map's view to the location of the new marker
      this.mapLeaflet.setView([msg.latitude, msg.longitude]);
  }

  // Function to clear marker history
  clearMarkerHistory() {
    for (const marker of this.markerHistory) {
      this.mapLeaflet.removeLayer(marker);
    }
    this.markerHistory = [];
  }
}

MapViewer.friendlyName = "Street Map";

MapViewer.supportedTypes = [
    "sensor_msgs/msg/NavSatFix",
];

MapViewer.maxUpdateRate = 10.0;

Viewer.registerViewer(MapViewer);