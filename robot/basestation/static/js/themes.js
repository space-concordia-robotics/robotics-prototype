$(document).ready(() => {

    function append_css(file) {
        console.log('Append CSS: ' + file);

        var link = document.createElement("link");
        link.href = '/static/css/' + file + ".css";
        link.type = "text/css";
        link.rel = "stylesheet";
        link.media = "screen,print";

        document.getElementsByTagName("head")[0].appendChild(link);
    }

    function prefer_theme(name) {
        console.log('Prefer theme: ' + name);
        setCookie('theme', name, 365); // setCookie() from helpers.js
        append_css('themes/' + name);
    }

    preferred_theme = getCookie('theme'); // getCookie() from helpers.js
    if (preferred_theme != "") {
        prefer_theme(preferred_theme);
    }

    $('#theme-light').click(function () {
        console.log('theme-light');
        prefer_theme('light');
    });
    $('#theme-dark').click(function () {
        console.log('theme-dark');
        prefer_theme('dark');
    });
});
