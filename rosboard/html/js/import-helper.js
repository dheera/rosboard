let importedPaths = {};

function importCssOnce(path) {
  if(path in importedPaths) return;
  $('<link>').appendTo('head').attr({
    type: 'text/css',
    rel: 'stylesheet',
    href: path
  });
  importedPaths[path] = 1;
}

function importJsOnce(path) {
  if(path in importedPaths) return;
  var result = $.ajax({ url: path, dataType: "script", async: false })
  if(result.status === 200) {
    importedPaths[path] = 1;
  } else {
    console.log(result.status + " error while importing " + path);
  }
}
