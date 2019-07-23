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
        var futuredate = new Date();
        var futuretime = futuredate.getTime();
        futuretime += 3600 * 1000 * 24 * 365; // Roughly 1 year into the future
        futuredate.setTime(futuretime);
        document.cookie = "theme=" + name + "; expires=" + futuredate.toUTCString() + "; path=/";
        append_css('themes/' + name);
    }

    function get_cookie(cname) {
        var name = cname + "=";
        var decodedCookie = decodeURIComponent(document.cookie);
        var ca = decodedCookie.split(';');
        for (var i = 0; i < ca.length; i++) {
            var c = ca[i];
            while (c.charAt(0) == ' ') {
                c = c.substring(1);
            }
            if (c.indexOf(name) == 0) {
                return c.substring(name.length, c.length);
            }
        }
        return "";
    }

    preferred_theme = get_cookie('theme');
    if (preferred_theme != "") {
        prefer_theme(preferred_theme);
    }

    $('#theme-red').click(function () {
        prefer_theme('red');
    });
    $('#theme-green').click(function () {
        console.log('theme-green');
        prefer_theme('green');
    });
    $('#theme-blue').click(function () {
        console.log('theme-blue');
        prefer_theme('blue');
    });
});

