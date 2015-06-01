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
      <div class="Button Green" id="{{component_name}}_show">Show</div>
    </div>
    <div class="ButtonsColumn">
      <div class="StartButton" id="{{component_name}}_start">START</div>
      <div class="Button Yellow" id="{{component_name}}_log">Log</div>
      <!--<div class="Button Orange" id="{{component_name}}_pause">Pause</div>-->
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
  $("div#{{component_name}}_log").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} log"});
  });
  $("div#{{component_name}}_show").click(function() {
    send_to_server({msg: "COMMAND {{component_name}} show"});
  });
</script>
