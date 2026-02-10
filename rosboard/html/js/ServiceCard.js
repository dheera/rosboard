"use strict";

class ServiceCard {
  /**
    * ServiceCard constructor.
    * @constructor
  **/
  constructor(card, serviceName, serviceType) {
    this.card = card;
    this.serviceName = serviceName;
    this.serviceType = serviceType;
    this.pendingRequests = new Map(); // Track pending service calls

    let that = this;

    // Add service response handler to card
    card.handleServiceResponse = function(response) {
      that.handleServiceResponse(response);
    };

    // div container at the top right for all the buttons
    card.buttons = $('<div></div>').addClass('card-buttons').text('').appendTo(card);

    // card title div
    card.title = $('<div></div>')
      .addClass('card-title')
      .html(`<i class="material-icons" style="vertical-align: middle; margin-right: 8px;">settings</i>${serviceName}`)
      .appendTo(card);

    // card content div
    card.content = $('<div></div>').addClass('card-content').appendTo(card);

    // Service info section
    let serviceInfo = $('<div></div>')
      .addClass('service-info')
      .css({
        'padding': '16px',
        'background-color': '#424242',
        'border-radius': '4px',
        'margin-bottom': '16px'
      })
      .appendTo(card.content);

    $('<div></div>')
      .css({'font-weight': 'bold', 'color': '#ffffff', 'margin-bottom': '8px'})
      .text('Service Information')
      .appendTo(serviceInfo);

    $('<div></div>')
      .css({'color': '#cccccc', 'font-size': '12px'})
      .html(`<strong>Type:</strong> ${serviceType}`)
      .appendTo(serviceInfo);

    $('<div></div>')
      .css({'color': '#cccccc', 'font-size': '12px', 'margin-top': '4px'})
      .html(`<strong>Name:</strong> ${serviceName}`)
      .appendTo(serviceInfo);

    // Service call section
    let callSection = $('<div></div>')
      .addClass('service-call-section')
      .appendTo(card.content);

    $('<div></div>')
      .css({'font-weight': 'bold', 'color': '#ffffff', 'margin-bottom': '12px'})
      .text('Service Call')
      .appendTo(callSection);

    // Request input
    $('<label></label>')
      .css({'display': 'block', 'color': '#cccccc', 'font-size': '12px', 'margin-bottom': '4px'})
      .text('Request (JSON):')
      .appendTo(callSection);

    card.requestInput = $('<textarea></textarea>')
      .addClass('service-request-input')
      .attr('placeholder', 'Enter service request in JSON format...\ne.g., {"a": 1, "b": 2}')
      .css({
        'width': '100%',
        'min-height': '80px',
        'font-family': 'JetBrains Mono, monospace',
        'font-size': '12px',
        'padding': '8px',
        'background-color': '#404040',
        'color': '#f0f0f0',
        'border': '1px solid #666666',
        'border-radius': '4px',
        'resize': 'vertical',
        'box-sizing': 'border-box',
        'margin-bottom': '8px'
      })
      .appendTo(callSection);

    // Call button
    card.callButton = $('<button></button>')
      .addClass('mdl-button')
      .addClass('mdl-js-button')
      .addClass('mdl-button--raised')
      .addClass('mdl-button--colored')
      .css({'margin-right': '8px'})
      .text('Call Service')
      .appendTo(callSection);

    // Clear button
    card.clearButton = $('<button></button>')
      .addClass('mdl-button')
      .addClass('mdl-js-button')
      .text('Clear')
      .appendTo(callSection);

    // Response section
    card.responseSection = $('<div></div>')
      .css({'margin-top': '16px'})
      .appendTo(callSection);

    $('<label></label>')
      .css({'display': 'block', 'color': '#cccccc', 'font-size': '12px', 'margin-bottom': '4px'})
      .text('Response:')
      .appendTo(card.responseSection);

    card.responseOutput = $('<pre></pre>')
      .css({
        'width': '100%',
        'min-height': '60px',
        'max-height': '200px',
        'font-family': 'JetBrains Mono, monospace',
        'font-size': '12px',
        'padding': '8px',
        'background-color': '#2e2e2e',
        'color': '#e0e0e0',
        'border': '1px solid #555555',
        'border-radius': '4px',
        'overflow-y': 'auto',
        'white-space': 'pre-wrap',
        'box-sizing': 'border-box',
        'margin-bottom': '8px'
      })
      .text('Service response will appear here...')
      .appendTo(card.responseSection);

    // Button event handlers
    card.callButton.click(function() {
      that.callService();
    });

    card.clearButton.click(function() {
      card.requestInput.val('');
      card.responseOutput.text('Service response will appear here...');
    });

    // Allow Ctrl+Enter to call service
    card.requestInput.keydown(function(e) {
      if (e.ctrlKey && e.key === 'Enter') {
        that.callService();
      }
    });

    // card close button
    card.closeButton = $('<button></button>')
      .addClass('mdl-button')
      .addClass('mdl-js-button')
      .addClass('mdl-button--icon')
      .append($('<i></i>').addClass('material-icons').text('close'))
      .appendTo(card.buttons);
    card.closeButton.click(() => { 
      if (ServiceCard.onClose) ServiceCard.onClose(that); 
    });

    // Initialize MDL components
    if(typeof(componentHandler) !== 'undefined') {
      componentHandler.upgradeAllRegistered();
    }
  }

  callService() {
    const requestText = this.card.requestInput.val().trim();
    
    console.log('ServiceCard.callService() called');
    console.log('currentTransport available:', typeof window.currentTransport !== 'undefined');
    console.log('currentTransport:', window.currentTransport);
    
    try {
      let requestData = {};
      if (requestText) {
        requestData = JSON.parse(requestText);
      }

      // Generate unique request ID 
      const requestId = 'req_' + Date.now() + '_' + Math.random().toString(36).substring(7);
      
      // Store request for response matching
      this.pendingRequests.set(requestId, {
        timestamp: Date.now(),
        request: requestData
      });

      // Update response to show calling status
      this.card.responseOutput
        .css({'color': '#81c784'})
        .text(`Calling service ${this.serviceName}...\nRequest ID: ${requestId}`);

      // Call service via transport - use window.currentTransport for global access
      if (typeof window.currentTransport !== 'undefined' && window.currentTransport && typeof window.currentTransport.callService === 'function') {
        console.log('Calling service via transport:', {
          serviceName: this.serviceName,
          serviceType: this.serviceType,
          request: requestData,
          requestId: requestId
        });
        
        window.currentTransport.callService({
          serviceName: this.serviceName,
          serviceType: this.serviceType,
          request: requestData,
          requestId: requestId
        });
        
        console.log(`Called service ${this.serviceName} with request:`, requestData);
        
        // Set timeout to handle case where service doesn't respond
        setTimeout(() => {
          if (this.pendingRequests.has(requestId)) {
            this.pendingRequests.delete(requestId);
            this.card.responseOutput
              .css({'color': '#e57373'})
              .text(`Service call timed out.\nRequest ID: ${requestId}\nNo response received within 30 seconds.`);
          }
        }, 30000);
        
      } else {
        let errorMsg = 'Service transport not available.';
        if (typeof window.currentTransport === 'undefined') {
          errorMsg += ' Transport is undefined.';
        } else if (!window.currentTransport) {
          errorMsg += ' Transport is null.';
        } else if (typeof window.currentTransport.callService !== 'function') {
          errorMsg += ' callService method not found.';
        }
        
        console.error(errorMsg);
        this.card.responseOutput
          .css({'color': '#e57373'})
          .text(errorMsg + ' Please check connection.');
      }

    } catch (e) {
      console.error('Error in callService:', e);
      this.card.responseOutput
        .css({'color': '#e57373'})
        .text(`Invalid JSON format: ${e.message}`);
    }
  }

  handleServiceResponse(response) {
    // Check if this response is for this service
    if (response.serviceName !== this.serviceName) {
      return;
    }

    const requestId = response.requestId;
    if (!requestId || !this.pendingRequests.has(requestId)) {
      return;
    }

    // Remove from pending requests
    this.pendingRequests.delete(requestId);

    // Display response
    if (response.success) {
      this.card.responseOutput
        .css({'color': '#81c784'})
        .text(`Service call successful!\nRequest ID: ${requestId}\n\nResponse:\n${JSON.stringify(response.response, null, 2)}`);
    } else {
      this.card.responseOutput
        .css({'color': '#e57373'})
        .text(`Service call failed!\nRequest ID: ${requestId}\n\nError: ${response.error}`);
    }
  }

  destroy() {
    this.card.empty();
  }
}

// Static callback for close events
ServiceCard.onClose = null;