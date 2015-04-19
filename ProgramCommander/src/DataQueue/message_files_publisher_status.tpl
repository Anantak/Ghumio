<div class="ComponentPanel" id="{{component_name}}">
  <div class="MessagesContainer" id="{{component_name}}_messages">
    <div class="PanelHeader" id="{{component_name}}_header">{{component_name}}</div>
    <div class="Message" id="{{component_name}}_status">{{status}}</div>
    <div class="Time" id="{{component_name}}_up_time">{{up_time}}</div>
    <div class="ProcessId" id="{{component_name}}_process_id">{{process_id}}</div>
    <div class="StatusString" id="{{component_name}}_status_str">{{status_str}}</div>
  </div>
  <div class="ButtonsContainer" id="{{component_name}}_buttons">
    <div class="ButtonsColumn">
      <div class="Button Green" id="{{component_name}}_play">PLAY</div>
      <div class="Button Orange" id="{{component_name}}_pause">PAUSE</div>      
      <div class="Button Red" id="{{component_name}}_stop">STOP</div>      
    </div>
    <div class="ButtonsColumn">
      <div class="StartButton" id="{{component_name}}_start">START</div>
      <div class="ExitButton" id="{{component_name}}_exit">EXIT</div>      
    </div>
  </div>
</div>
<script>
  function update_{{component_name}}(data) {
    $("div#{{component_name}}_status").text(data.status);
    $("div#{{component_name}}_up_time").text(data.up_time);
    $("div#{{component_name}}_process_id").text(data.process_id);
    $("div#{{component_name}}_status_str").text(data.status_str);
  }
  $("div#{{component_name}}_start").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} start"});
  });
  $("div#{{component_name}}_exit").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} exit"});
  });
  $("div#{{component_name}}_play").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} play"});
  });
  $("div#{{component_name}}_stop").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} stop"});
  });
  $("div#{{component_name}}_pause").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} pause"});
  });
</script>
