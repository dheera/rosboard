"use strict";


class GenericPublisher extends Publisher {

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
        $('<td></td>').text('data').css(
                      {'width': '40%',
                       'font-weight': 'bold',
                       'overflow': 'hidden', 
                       'text-overflow': 'ellipsis'
                      }),

        $('<td></td>').append(
          $('<textarea></textarea>')
			      .attr('type', 'text')
			      .attr('rows', '1')
		        .attr('placeholder', 'JSON Data')
			      .addClass('mdl-textfield__input')
        )
      ).appendTo(this.dataTable);

    super.onCreate();
  }


  asJSON() {
	  return {data: JSON.parse(this.dataTable.find('textarea').val())};
  }

  beforePublishing() {
	  super.beforePublishing();
	  this.dataTable.find('textarea').prop('disabled', true);
  }

  afterPublishing() {
	  super.afterPublishing();
  	this.dataTable.find('textarea').prop('disabled', false);
  }
}

GenericPublisher.friendlyName = "Raw data";
GenericPublisher.supportedTypes = ["*"];
Publisher.registerPublisher(GenericPublisher);
