var materialAlert = function(text) {
    let dialog = $('<dialog></dialog>')
        .addClass('mdl-dialog')
        .css({
            'width': '50vw',
            'margin-left': '-25vw',
            'left': '50%'
        });

    let content = $('<div></div>')
        .addClass('mdl-dialog__content')
        .text(text)
        .appendTo(dialog);

    let actions = $('<div></div>')
        .addClass('mdl-dialog__actions')
        .appendTo(dialog);

    let buttonOK = $('<button></button>')
        .addClass('mdl-button mdl-js-button mdl-button--raised mdl-button--colored')
        .text('OK')
        .click(function() {
            dialog[0].close();
        })
        .appendTo(actions);

    dialog.appendTo(document.body);
    dialog[0].showModal();
};

