"use strict";

// Publisher is just a base class. It just has the boilerplate code to
// instantiate the elemnets (title, content, close button, spinner) of a card
// and display an error if there is an error. Publisher doesn't have any visualization
// capability at all, hence, it has no supportedTypes. Child classes will inherit
// from Publisher and implement visualization functionality.

class Publisher {
  /**
    * Class constructor.
    * @constructor
  **/
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

    let menuId = 'menu-' + Math.floor(Math.random() * 1e6);

    card.settingsButton = $('<button id="' + menuId + '"></button>')
    .addClass('mdl-button')
    .addClass('mdl-js-button')
    .addClass('mdl-button--icon')
    .addClass('mdl-button--colored')
    .append($('<i></i>').addClass('material-icons').text('more_vert'))
    .appendTo(card.buttons);
    
    card.menu = $('<ul class="mdl-menu mdl-menu--bottom-right mdl-js-menu mdl-js-ripple-effect" \
      for="' + menuId + '"></ul>').appendTo(card);
   
    // card close button
    card.closeButton = $('<button></button>')
      .addClass('mdl-button')
      .addClass('mdl-js-button')
      .addClass('mdl-button--icon')
      .append($('<i></i>').addClass('material-icons').text('close'))
      .appendTo(card.buttons);
    card.closeButton.click(() => { Publisher.onClose(that); });

    card.footer_buttons = $('<div></div>').addClass('card-buttons-footer').text('').appendTo(card.footer);
    card.continuousSendDiv = $('<div></div>').addClass('card-continuous_send').text('').appendTo(card.footer);
    card.continuousSendDivText = $('<div></div>').addClass('card-continuous_send-title').text('Continuous Send').appendTo(card.continuousSendDiv);
    card.continuousSendCheck = $('<input>', {type: 'checkbox'})
		  .appendTo(card.continuousSendDiv);
    card.continuousSendCheck.change(function () {that.continuousSend = this.checked;});


    card.sendButton = $('<button></button>')
      .addClass("mdl-button")
      .addClass('mdl-js-button')
      .css('color', '#a0a0a0')
      .css('border', '1px solid')
      .text('Publish')
      .appendTo(card.footer_buttons);
    card.sendButton.click(() => { Publisher.onPublish(that); });



    let publishers = Publisher.getPublishersForType(this.topicType);
    for(let i in publishers) {
      let item = $('<li ' + (publishers[i].name === this.constructor.name ? 'disabled' : '') + ' class="mdl-menu__item">' + publishers[i].friendlyName + '</li>').appendTo(this.card.menu);
      let that = this;
      item.click(() => { Publisher.onSwitchPublisher(that, publishers[i]); });
    }

    componentHandler.upgradeAllRegistered();


    // call onCreate(); child class will override this and initialize its UI
    this.onCreate();

    this.lastDataTime = 0.0;
  }

  /**
    * Gets called when Publisher is first initialized.
    * @override
  **/
  onCreate() {
    // for MDL elements to get instantiated
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
	  this.card.sendButton.text('Stop');
	  this.card.continuousSendCheck.prop('disabled', true);
  }
  afterPublishing()  {
	  this.isPublishing = false;
	  this.card.sendButton.text('Publish');
	  this.card.continuousSendCheck.prop('disabled', false);
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

// can be overridden by child class
// list of supported message types by viewer, or "*" for all types
// todo: support regexes?
Publisher.supportedTypes = [];

// can be overridden by child class
// max update rate that this viewer can handle
// for some viewers that do extensive DOM manipulations, this should be set conservatively
Publisher.maxSendRate = 50.0;


// not to be overwritten by child class!
// stores registered viewers in sequence of loading
Publisher._publishers = [];

// override this
Publisher.onClose = (publisherInstance) => { console.log("not implemented; override necessary"); }
// On button click - to override
Publisher.onPublish = (publisherInstance) => { console.log("not implemented; override necessary"); }



Publisher.onSwitchPublisher = (publisherInstance, newPublisherType) => { console.log("not implemented; override necessary"); }

// not to be overwritten by child class!
Publisher.registerPublisher = (publisher) => {
  // registers a viewer. the viewer child class calls this at the end of the file to register itself
  Publisher._publishers.push(publisher);
};

// not to be overwritten by child class!
Publisher.getDefaultPublisherForType = (type) => {
  // gets the viewer class for a given message type (e.g. "std_msgs/msg/String")

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

