"use strict";

class Publisher {
  constructor(card, topicName, topicType) {
    this.card = card;

    this.topicName = topicName;
    this.topicType = topicType;

    this.continuousSend = false;
    this.isPublishing   = false;
    this.publishInterval = null;

    this.onClose = () => {};
    let that = this;

    // div container at the top right for all the buttons
    card.buttons = $('<div></div>').addClass('card-buttons').text('').appendTo(card);
    card.title = $('<div></div>').addClass('card-title').text(topicName + " - " + topicType + " - " + "[pub]").appendTo(card);

    card.content = $('<div></div>').addClass('card-content').text('').appendTo(card);
    card.footer = $('<div></div>').addClass('card-footer').text('').appendTo(card);

    card.closeButton = $('<button></button>')
      .addClass('mdl-button')
      .addClass('mdl-js-button')
      .addClass('mdl-button--icon')
      .append($('<i></i>').addClass('material-icons').text('close'))
      .click(() => { Publisher.onClose(that); })
      .appendTo(card.buttons);

    card.continuous_send = $('<input>', {type: 'checkbox', id: 'continuous_send'})
      .change(function () { that.continuousSend = this.checked;})
      .addClass("mdl-checkbox__input")
			.addClass("footer_checkbox");
    
    card.rate = $('<input>', {type: 'number'})
	  	.addClass("mdl-textfield__input")
	  	.addClass("footer_number")
	  	.prop("value",1)
	  	.prop("min", 0.1)
	 	  .prop("max", 100)
	  	.prop("step", 0.1)
     	.css('flex', '1 1 1')
	 		.on('beforeinput', function (e) {
			  const input = e.target.value;
			  const data = e.originalEvent.data || "";
			  const pattern = /^[0-9]*(\.[0-9]*)?$/;

			  if(e.originalEvent.inputType === "deleteContentBackward") return;
			  if(!pattern.test(input + data)) e.preventDefault();
		  });

    card.publishButton = $('<button></button>')
   		.addClass("mdl-button")
   		.addClass('mdl-js-button')
   		.css('color', '#a0a0a0')
   		.css('border', '1px solid')
   		.css('flex', '1 1 1')
    	.text('Publish')
	 		.click(() => { Publisher.onPublish(that); });

     
    $('<div></div>').addClass('card-footer_content')
		  .text('')
		  .appendTo(card.footer)
		  .append(
  			card.continuous_send,
        $('<span>', {'class': 'mdl-checkbox__label', text: 'Continuous Send'})
		  );

    $('<div></div>').addClass('card-footer_content')
		  .text('')
		  .appendTo(card.footer)
		  .append(
			  card.rate,
    		$('<span>', {'class': 'mdl-checkbox__label', text: 'Rate (Hz)'})
		  );

    $('<div></div>').addClass('card-footer_content')
		  .text('')
		  .appendTo(card.footer)
		  .append(card.publishButton);

   

    let publishers = Publisher.getPublishersForType(this.topicType);
    for(let i in publishers) {
      let item = $('<li ' + (publishers[i].name === this.constructor.name ? 'disabled' : '') + ' class="mdl-menu__item">' + publishers[i].friendlyName + '</li>').appendTo(this.card.menu);
      let that = this;
      item.click(() => { Publisher.onSwitchPublisher(that, publishers[i]); });
    }

    componentHandler.upgradeAllRegistered();

    this.onCreate();
    this.lastDataTime = 0.0;
  }

  onCreate() {
    if(!(typeof(componentHandler) === 'undefined')){
      componentHandler.upgradeAllRegistered();
    }
  }

  destroy() {
    this.card.empty();
  }

  onResize() { }

  asJSON() {}

  beforePublishing() {
	  this.isPublishing = true;
	  this.card.publishButton.text('Stop');
	  this.card.continuous_send.prop('disabled', true);
	  this.card.rate.prop('disabled', true);
  }

  afterPublishing()  {
	  this.isPublishing = false;
	  this.card.publishButton.text('Publish');
	  this.card.continuous_send.prop('disabled', false);
	  this.card.rate.prop('disabled', false);
  }

  warn(warn_text) {
    if(!this.card.warn) {
      this.card.warn = $("<div></div>").css({
        "background": "#a08000",
        "color": "#ffffff",
        "padding": "20pt",
      }).appendTo(this.card);
    }
    this.card.warn.text(warn_text).css({
      "display": "",
    });
  }

  tip(tip_text) {
    if(this.tipHideTimeout) clearTimeout(this.tipHideTimeout);
    if(!this.tipBox) {
      this.tipBox = $("<div></div>").css({
        "background": "rgba(0,0,0,0.3)",
        "position": "absolute",
        "z-index": "10",
        "bottom": "0",
        "width": "calc( 100% - 24pt )",
        "height": "24px",
        "text-overflow": "ellipsis",
        "overflow": "hidden",
        "padding-left": "12pt",
        "padding-right": "12pt",
        "padding-bottom": "4pt",
        "padding-top": "4pt",
        "font-size": "8pt",
        "white-space": "nowrap",
        "color": "#ffffff",
      }).addClass("monospace").appendTo(this.card);
    }
    let that = this;
    this.tipBox.css({"display": ""});
    this.tipHideTimeout = setTimeout(() => that.tipBox.css({"display": "none"}), 1000);
    this.tipBox.text(tip_text);
  }
}

Publisher.friendlyName = "Publisher";

Publisher.supportedTypes = [];

// not to be overwritten by child class!
Publisher._publishers = [];

Publisher.onClose = (publisherInstance) => { console.log("not implemented; override necessary"); }
Publisher.onPublish = (publisherInstance) => { console.log("not implemented; override necessary"); }

Publisher.onSwitchPublisher = (publisherInstance, newPublisherType) => { console.log("not implemented; override necessary"); }

Publisher.registerPublisher = (publisher) => {
  Publisher._publishers.push(publisher);
};

Publisher.getDefaultPublisherForType = (type) => {
  // gets the publisher class for a given message type (e.g. "std_msgs/msg/String")
  // if type is "package/MessageType", converted it to "package/msgs/MessageType"
  let tokens = type.split("/");
  if(tokens.length == 2) {
    type = [tokens[0], "msg", tokens[1]].join("/");
  }

  // go down the list of registered viewers and return the first match
  for(let i in Publisher._publishers) {
    if(Publisher._publishers[i].supportedTypes.includes(type)) {
      return Publisher._publishers[i];
    }
    if(Publisher._publishers[i].supportedTypes.includes("*")) {
      return Publisher._publishers[i];
    }
  }
  return null;
}

// not to be overwritten by child class!
Publisher.getPublishersForType = (type) => {
  // gets the viewer classes for a given message type (e.g. "std_msgs/msg/String")

  let matchingPublishers = [];

  // if type is "package/MessageType", converted it to "package/msgs/MessageType"
  let tokens = type.split("/");
  if(tokens.length == 2) {
    type = [tokens[0], "msg", tokens[1]].join("/");
  }

  // go down the list of registered viewers and return the first match
  for(let i in Publisher._publishers) {
    if(Publisher._publishers[i].supportedTypes.includes(type)) {
      matchingPublishers.push(Publisher._publishers[i]);
    }
    if(Publisher._publishers[i].supportedTypes.includes("*")) {
      matchingPublishers.push(Publisher._publishers[i]);
    }
  }

  return matchingPublishers;
}

