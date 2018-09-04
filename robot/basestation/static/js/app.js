$(function() {
    const Site = {
        init: function() {
            this.bindEventHandlers();
        },
        bindEventHandlers: function() {
            this.eventHandlers.forEach(eventHandler => this.bindEvent(eventHandler));
        },
        bindEvent: function(e) {
            const binder = function() {
                $.getJSON(e.route, e.handler);
                intervalId = setInterval(function() {$.getJSON(e.route, e.handler);}, 100);
                return false;
            };
            e.$el.bind(e.event, binder);
            e.$el.bind("mouseup", function() {
                clearInterval(intervalId);
            });
            
            console.log('Bound ' + e.event + ' handler for', e.$el);

        },
        eventHandlers: [
            {
                $el: $('a#btn_pitch_up'),
                event: "mousedown",
                route: "/mousedown_btn_pitch_up",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_pitch_down'),
                event: "mousedown",
                route: "/mousedown_btn_pitch_down",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_roll_left'),
                event: "mousedown",
                route: "/mousedown_btn_roll_left",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_roll_right'),
                event: "mousedown",
                route: "/mousedown_btn_roll_right",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_claw_open'),
                event: "mousedown",
                route: "/mousedown_btn_claw_open",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_claw_close'),
                event: "mousedown",
                route: "/mousedown_btn_claw_close",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_arm_up'),
                event: "mousedown",
                route: "/mousedown_btn_arm_up",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_arm_down'),
                event: "mousedown",
                route: "/mousedown_btn_arm_down",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_arm_left'),
                event: "mousedown",
                route: "/mousedown_btn_arm_left",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_arm_right'),
                event: "mousedown",
                route: "/mousedown_btn_arm_right",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_arm_back'),
                event: "mousedown",
                route: "/mousedown_btn_arm_back",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_arm_forward'),
                event: "mousedown",
                route: "/mousedown_btn_arm_forward",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor1_ccw'),
                event: "mousedown",
                route: "/mousedown_btn_motor1_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor1_cw'),
                event: "mousedown",
                route: "/mousedown_btn_motor1_cw",
                handler: function(data) { 
                    setInterval(this, 1000);
                }
            },
            {
                $el: $('a#btn_motor2_ccw'),
                event: "mousedown",
                route: "/mousedown_btn_motor2_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor2_cw'),
                event: "mousedown",
                route: "/mousedown_btn_motor2_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor3_ccw'),
                event: "mousedown",
                route: "/mousedown_btn_motor3_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor3_cw'),
                event: "mousedown",
                route: "/mousedown_btn_motor3_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor4_ccw'),
                event: "mousedown",
                route: "/mousedown_btn_motor4_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor4_cw'),
                event: "mousedown",
                route: "/mousedown_btn_motor4_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor5_ccw'),
                event: "mousedown",
                route: "/mousedown_btn_motor5_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor5_cw'),
                event: "mousedown",
                route: "/mousedown_btn_motor5_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor6_ccw'),
                event: "mousedown",
                route: "/mousedown_btn_motor6_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#btn_motor6_cw'),
                event: "mousedown",
                route: "/mousedown_btn_motor6_cw",
                handler: function(data) { }
            },
        ]
    };
    
    Site.init();
})();