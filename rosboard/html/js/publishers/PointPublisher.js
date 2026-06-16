"use strict";

class PointPublisher extends Publisher {
  /**
    * Gets called when Viewer is first initialized.
    * @override
  **/
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

    $('<tr></tr>')
      .append(
        $('<td></td>').append(
		$('<input>')
			.attr('id', 'x')
			.attr('type', 'number')
		        .attr('placeholder', 'x')
			.addClass('mdl-textfield__input')
			.on('beforeinput', function (e) {
					const input = e.target.value;
					const data = e.originalEvent.data || "";
					const pattern = /^[0-9]*(\.[0-9]*)?$/;
					if(e.originalEvent.inputType === "deleteContentBackward") return;
					if(!pattern.test(input + data)) e.preventDefault();
				})

        ),
	$('<td></td>').append(
		$('<input>')
			.attr('id', 'y')
			.attr('type', 'number')
		        .attr('placeholder', 'y')
			.addClass('mdl-textfield__input')
			.on('beforeinput', function (e) {
					const input = e.target.value;
					const data = e.originalEvent.data || "";
					const pattern = /^[0-9]*(\.[0-9]*)?$/;
					if(e.originalEvent.inputType === "deleteContentBackward") return;
					if(!pattern.test(input + data)) e.preventDefault();
				})
        ),
	$('<td></td>').append(
		$('<input>')
			.attr('id', 'z')
			.attr('type', 'number')
		        .attr('placeholder', 'z')
			.addClass('mdl-textfield__input')
			.on('beforeinput', function (e) {
					const input = e.target.value;
					const data = e.originalEvent.data || "";
					const pattern = /^[0-9]*(\.[0-9]*)?$/;
					if(e.originalEvent.inputType === "deleteContentBackward") return;
					if(!pattern.test(input + data)) e.preventDefault();
				})
        ),
      )
  .appendTo(this.dataTable);
    super.onCreate();
  }


  asJSON() {
	return {
		x: parseFloat($('#x').val()),
		y: parseFloat($('#y').val()),
		z: parseFloat($('#z').val())
	};
  }

  beforePublishing() {
	super.beforePublishing();
	$("#x").prop('disabled', true);
	$("#y").prop('disabled', true);
	$("#z").prop('disabled', true);
  }

  afterPublishing() {
	super.afterPublishing();
	$("#x").prop('disabled', false);
	$("#y").prop('disabled', false);
	$("#z").prop('disabled', false);
  }

  
}

PointPublisher.friendlyName = "Point";

PointPublisher.supportedTypes = [
    "geometry_msgs/msg/Point",
];

Publisher.registerPublisher(PointPublisher);
