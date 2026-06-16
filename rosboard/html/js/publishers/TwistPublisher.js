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

class TwistPublisher extends Publisher {
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
   ["linear", "angular"].forEach((_class) => {
     $('<tr></tr>')
      .append(
        $('<td></td>')
	  .text(_class)
          .css({'width': '40%', 'font-weight': 'bold', 'overflow': 'hidden', 'text-overflow': 'ellipsis'}),
        $('<td></td>').append(createFloatInput(_class, "x")),
        $('<td></td>').append(createFloatInput(_class, "y")),
        $('<td></td>').append(createFloatInput(_class, "z")),
      )
  .appendTo(this.dataTable);
   });


  super.onCreate();
  }


  asJSON() {
	return {
		"linear": {
			"x": parseFloat($('#linear_x').val()),
			"y": parseFloat($('#linear_y').val()),
			"z": parseFloat($('#linear_z').val())
	 	},
		"angular": {
			"x": parseFloat($('#angular_x').val()),
			"y": parseFloat($('#angular_y').val()),
			"z": parseFloat($('#angular_z').val())
	 	}
  	};
  }

  beforePublishing() {
	super.beforePublishing();
	["linear", "angular"].forEach((_class) => {
		["x", "y", "z"].forEach((id) => {$("#" + _class + "_" + id).prop('disabled', true);});
	});

  }

  afterPublishing() {
	super.afterPublishing();
	["linear", "angular"].forEach((_class) => {
		["x", "y", "z"].forEach((id) => {$("#" + _class + "_" + id).prop('disabled', true);});
	});

  }

  
}

TwistPublisher.friendlyName = "Twist";

TwistPublisher.supportedTypes = [
    "geometry_msgs/msg/Twist",
];

Publisher.registerPublisher(TwistPublisher);
