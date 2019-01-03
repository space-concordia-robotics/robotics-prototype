/* eslint-disable no-unused-vars */
$(document).ready(() => {
  const Site = {
    init() {
      this.bindEventHandlers();
    },
    bindEventHandlers() {
      this.eventHandlers.forEach(eventHandler => this.bindEvent(eventHandler));
    },
    bindEvent(e) {
      let intervalId;
      const timeoutMs = 100;
      const $callEndpointOnEvent = () => $.getJSON(e.route, e.handler);
      const loopCallEndpointUntilEventDone = () => { 
        // Call once immediately as event gets fired.
        $callEndpointOnEvent(); 
        // Call starts after timeoutMs and loops until event no longer triggered.
        intervalId = setInterval($callEndpointOnEvent, timeoutMs);  
      };
      const stopCallEndpointWhenEventDone = () => { clearInterval(intervalId); };

      e.$el.on(e.event, loopCallEndpointUntilEventDone);
      e.$el.on('mouseup', stopCallEndpointWhenEventDone);

      console.log('Bound ' + e.event + ' handler for', e.$el);
    },
    eventHandlers: [
      {
        $el: $('#btn_pitch_up'),
        event: 'mousedown',
        route: '/mousedown_btn_pitch_up',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_pitch_down'),
        event: 'mousedown',
        route: '/mousedown_btn_pitch_down',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_roll_left'),
        event: 'mousedown',
        route: '/mousedown_btn_roll_left',
        handler: (data) => { console.log(data) },
      },
      {
        $el: $('#btn_roll_right'),
        event: 'mousedown',
        route: '/mousedown_btn_roll_right',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_claw_open'),
        event: 'mousedown',
        route: '/mousedown_btn_claw_open',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_claw_close'),
        event: 'mousedown',
        route: '/mousedown_btn_claw_close',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_arm_up'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_up',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_arm_down'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_down',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_arm_left'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_left',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_arm_right'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_right',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_arm_backward'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_backward',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_arm_forward'),
        event: 'mousedown',
        route: '/mousedown_btn_arm_forward',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor1_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor1_ccw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor1_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor1_cw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor2_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor2_ccw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor2_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor2_cw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor3_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor3_ccw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor3_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor3_cw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor4_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor4_ccw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor4_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor4_cw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor5_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor5_ccw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor5_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor5_cw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor6_ccw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor6_ccw',
        handler: (data) => { console.log(data); },
      },
      {
        $el: $('#btn_motor6_cw'),
        event: 'mousedown',
        route: '/mousedown_btn_motor6_cw',
        handler: (data) => { console.log(data); },
      },
    ],
  };

  Site.init();
});
