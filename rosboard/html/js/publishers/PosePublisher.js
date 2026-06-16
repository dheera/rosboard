"use strict";

function createFloatInput(_class, id){
	return $('<input>')
			.attr('id', _class + '_' + id)
			.attr('type', 'number')
		        .attr('placeholder', id)
			.addClass('mdl-textfield__input')
			.on('beforeinput', function (e) {
					const input = e.target.value;
					const data = e.originalEvent.data || "";
					const pattern = /^[0-9]*(\.[0-9]*)?$/;
					if(e.originalEvent.inputType === "deleteContentBackward") return;
					if(!pattern.test(input + data)) e.preventDefault();
				})

  }

class PosePublisher extends Publisher {
  onCreate() {
    this.publisherNode = $('<div></div>')
      .css({'font-size': '11pt'})
      .appendTo(this.card.content);

    this.publisherNodeFadeTimeout = null;

    this.expandFields = { };
    this.fieldNodes = { };
    this.dataTable = $('<table></table>')
          .addClass('mdl-data-table')
          .addClass('mdl-js-data-table')
          .css({'width': '100%', 'min-height': '30pt', 'table-layout': 'fixed' })
          .appendTo(this.publisherNode);
   ["position", "orientation"].forEach((_class) => {
     var tr = $('<tr></tr>')
      .append(
        $('<td></td>')
	  .text(_class)
          .css({'width': '30%', 'font-weight': 'bold', 'overflow': 'hidden', 'text-overflow': 'ellipsis'}),
        $('<td></td>').append(createFloatInput(_class, "x")),
        $('<td></td>').append(createFloatInput(_class, "y")),
        $('<td></td>').append(createFloatInput(_class, "z")),
      )
  .appendTo(this.dataTable);

  if (_class === "orientation") tr.append($('<td></td>').append(createFloatInput(_class, "w")));
   });




  super.onCreate();
  }


  asJSON() {
	return {
		"position": {
			"x": parseFloat($('#position_x').val()),
			"y": parseFloat($('#position_y').val()),
			"z": parseFloat($('#position_z').val())
	 	},
		"orientation": {
			"x": parseFloat($('#orientation_x').val()),
			"y": parseFloat($('#orientation_y').val()),
			"z": parseFloat($('#orientation_z').val()),
			"w": parseFloat($('#orientation_w').val())
	 	}
  	};
  }

  beforePublishing() {
	super.beforePublishing();
	["position", "orientation"].forEach((_class) => {
		["x", "y", "z"].forEach((id) => {$("#" + _class + "_" + id).prop('disabled', true);});
	});

  }

  afterPublishing() {
	super.afterPublishing();
	["position", "orientation"].forEach((_class) => {
		["x", "y", "z"].forEach((id) => {$("#" + _class + "_" + id).prop('disabled', true);});
	});

  }

  
}

PosePublisher.friendlyName = "Pose";

PosePublisher.supportedTypes = [
    "geometry_msgs/msg/Pose",
];

Publisher.registerPublisher(PosePublisher);
