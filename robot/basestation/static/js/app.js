$(function() {
    const Site = {
        init: function() {
            this.bindEventHandlers();
        },
        bindEventHandlers: function() {
            this.eventHandlers.forEach(eventHandler => this.bindEvent(eventHandler));
        },
        bindEvent: function(e) {
            e.$el.bind(e.event, function() {
                $.getJSON(e.route, e.handler);
                return false;
            });
            console.log('Bound ' + e.event + ' handler for', e.$el);
        },
        eventHandlers: [
            {
                $el: $('a#click_btn_pitch_up'),
                event: "click",
                route: "/click_btn_pitch_up",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_pitch_down'),
                event: "click",
                route: "/click_btn_pitch_down",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_roll_left'),
                event: "click",
                route: "/click_btn_roll_left",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_roll_right'),
                event: "click",
                route: "/click_btn_roll_right",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_claw_open'),
                event: "click",
                route: "/click_btn_claw_open",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_claw_close'),
                event: "click",
                route: "/click_btn_claw_close",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_arm_up'),
                event: "click",
                route: "/click_btn_arm_up",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_arm_down'),
                event: "click",
                route: "/click_btn_arm_down",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_arm_left'),
                event: "click",
                route: "/click_btn_arm_left",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_arm_right'),
                event: "click",
                route: "/click_btn_arm_right",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_arm_back'),
                event: "click",
                route: "/click_btn_arm_back",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_arm_forward'),
                event: "click",
                route: "/click_btn_arm_forward",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor1_ccw'),
                event: "click",
                route: "/click_btn_motor1_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor1_cw'),
                event: "click",
                route: "/click_btn_motor1_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor2_ccw'),
                event: "click",
                route: "/click_btn_motor2_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor2_cw'),
                event: "click",
                route: "/click_btn_motor2_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor3_ccw'),
                event: "click",
                route: "/click_btn_motor3_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor3_cw'),
                event: "click",
                route: "/click_btn_motor3_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor4_ccw'),
                event: "click",
                route: "/click_btn_motor4_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor4_cw'),
                event: "click",
                route: "/click_btn_motor4_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor5_ccw'),
                event: "click",
                route: "/click_btn_motor5_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor5_cw'),
                event: "click",
                route: "/click_btn_motor5_cw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor6_ccw'),
                event: "click",
                route: "/click_btn_motor6_ccw",
                handler: function(data) { }
            },
            {
                $el: $('a#click_btn_motor6_cw'),
                event: "click",
                route: "/click_btn_motor6_cw",
                handler: function(data) { }
            },
        ]
    };
    
    Site.init();
})();