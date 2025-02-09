// redirect from example to example/bundle for demo
// link backward compatibility only on github-hosted demo
const url = new URL(location.href);
const tokens = url.pathname.split(/[\\/]/g);
const filename = tokens.pop();
const parentDirectory = tokens[ tokens.length - 1 ];
if (url.origin.includes('github') && parentDirectory !== 'bundle') {
    url.pathname = tokens.join('/') + '/';
    window.location.replace(new URL('bundle/' + filename, url.toString()));
}