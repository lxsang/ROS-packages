

var VIZ = {
    uri: "ws://10.1.160.205:9090",
    handles: {},
    canvas: null,
    buffer: null,
    resolution: 0.05,
    robotradius: 0.25,
    origin: { x: 0, y: 0 },
    msgcount: 10,
    changed: false,
    rate: 500, // ms
    lock: false,
    msgs: {},
    events: {},
    scale: 1.0,
    draw: function () {
        VIZ.unbindAll();
        if (VIZ.msgs.map)
        {
            VIZ.drawMap(VIZ.msgs.map);
        }
        if (VIZ.msgs.path)
        {
            VIZ.drawPath(VIZ.msgs.path);
        }
        if (VIZ.msgs.pose)
        {
            VIZ.drawRobot(VIZ.msgs.pose);
        }
        if (VIZ.msgs.goals)
        {
            VIZ.drawGoals(VIZ.msgs.goals);
        }
        VIZ.changed = true;
    },
    bind: function (name, obj) {
        if (!VIZ.events[name]) {

            VIZ.events[name] = [];
            $(VIZ.canvas).on(name, function (e) {
                var offset = $(VIZ.canvas).offset();
                var mouse = {
                    x: (e.clientX - offset.left),
                    y: (e.clientY - offset.top)
                };
                var hit = function (p, box) {
                    var left = box.x * VIZ.scale;
                    var top = box.y * VIZ.scale;
                    var right = (box.x + box.w) * VIZ.scale;
                    var bottom = (box.y + box.h) * VIZ.scale;
                    return (p.x > left && p.y > top &&
                            p.x < right && p.y < bottom);
                };
                for (var i = 0; i < VIZ.events[name].length; i+=1) {
                    if (hit(mouse, VIZ.events[name][i].box)) {
                        VIZ.events[name][i].handle(e);
                    }
                }

            });
        }
        VIZ.events[name].push(obj);
    },
    unbindAll: function () {
        for(var k in VIZ.events)
        {
            $(VIZ.canvas).unbind(k);
        }
        VIZ.events = {};
    },
    ready: function (resolve, reject) {
        
        //subscriber to topic map
        //return new Promise(function (resolve, reject) {
            var ros = new ROSLIB.Ros({
                url: VIZ.uri
            });

            ros.on("connection", function () {
                alertify.success("Connected to websocket server.");
                resolve(ros);
            });

            ros.on("error", function (error) {
                reject("Error connecting to websocket server: " + VIZ.uri);
            });

            ros.on("close", function () {
                alertify.warning("Connection to websocket server closed.");
            });
            
        //});
    },
    init: function (resolve, reject) {
        //return new Promise(function (resolve, reject) {
            VIZ.ready(function (ros) {
                    VIZ.handles.maplistener = new ROSLIB.Topic({
                        ros: ros,
                        name: "/map",
                        messageType: "nav_msgs/OccupancyGrid",
                        throttle_rate: VIZ.rate
                    });

                    VIZ.handles.maplistener.subscribe(function (message) {
                        VIZ.msgs.map = message;
                        VIZ.draw();
                        //listener.unsubscribe();
                        //ros.close()
                    });

                    VIZ.handles.pathlistener = new ROSLIB.Topic({
                        ros: ros,
                        name: "/move_base/GlobalPlanner/plan",
                        messageType: "nav_msgs/Path",
                        throttle_rate: VIZ.rate
                    });
                    VIZ.handles.pathlistener.subscribe(function (msg) {
                        VIZ.msgs.path = msg;
                        VIZ.draw();
                    });

                    VIZ.handles.poselistener = new ROSLIB.Topic({
                        ros: ros,
                        name: "/robot_pose",
                        messageType: "geometry_msgs/Pose",
                        throttle_rate: VIZ.rate
                    });

                    VIZ.handles.poselistener.subscribe(function (msg) {
                        //console.log(msg)
                        VIZ.msgs.pose = msg;
                        VIZ.draw();
                    });

                    VIZ.handles.goallistener = new ROSLIB.Topic({
                        ros: ros,
                        name: "/robotcmd/goals",
                        messageType: "geometry_msgs/PoseArray",
                        throttle_rate: VIZ.rate
                    });

                    VIZ.handles.goallistener.subscribe(function (msg) {
                        //console.log(msg)
                        VIZ.msgs.goals = msg;
                        //console.log(msg)
                        VIZ.draw();
                    });

                    VIZ.handles.goalstatus = new ROSLIB.Topic({
                        ros: ros,
                        name: "/move_base/status",
                        messageType: "actionlib_msgs/GoalStatusArray",
                        throttle_rate: VIZ.rate
                    });
                    VIZ.handles.next_goal = new ROSLIB.Topic({
                        ros: ros,
                        name: "/robotcmd/next_goal",
                        messageType: "std_msgs/Bool"
                    });
                    VIZ.handles.goalstatus.subscribe(function (msg) {
                        //inform the user to go to next
                        if (VIZ.lock) return;
                        if (!VIZ.msgs.goals || VIZ.msgs.goals.poses.length == 0) return;
                        if(VIZ.msgcount > 0) { VIZ.msgcount--; return;  }
                        var busy = false;
                        for (var i = 0; i < msg.status_list.length; i++) {
                            if (msg.status_list[i].status != 3)
                                busy = true;
                        }
                        if (busy) return;
                        VIZ.lock = true;
                        alertify.alert().setting("modal", false);
                        alertify.alert("Closable: false").set("closable", false); 
                        alertify.alert("Go next", "Navigate to the next goal?", function(){ 
                            var msg = new ROSLIB.Message({data: true});
                            VIZ.handles.next_goal.publish(msg);
                            VIZ.lock = false;
                            VIZ.msgcount = 10;
                            alertify.success("Going to the next goal");
                        });

                    });

                    resolve();
                },
                function (e) {
                    reject(e);
                });
        //});
    },
    main: function () {
        VIZ.init(VIZ.update, function (e) {
            alertify.error(e);
            alertify.prompt("Please enter the correct Uri", VIZ.uri,
                function (evt, value) {
                    VIZ.uri = value;
                    VIZ.main();
                },
                function () {
                    alertify.error("Abort the connection");
                })
                ;
        });
    },
  robotPose2Canvas: function(p) {
        var center_x = VIZ.origin.x + p.x / VIZ.resolution;
        var center_y = VIZ.origin.y - p.y / VIZ.resolution;
        return { x: center_x, y: center_y };
    },
    drawGoals: function (msg) {
        //console.log("draw goal");
        var canvas = VIZ.buffer;
        var context = canvas.getContext("2d");
        var radius = 7;

        for (var i = 0; i < msg.poses.length; i++) {
            var pose = VIZ.robotPose2Canvas(msg.poses[i].position);
            context.moveTo(pose.x, pose.y);
            context.beginPath();
            //console.log(radius);
            context.arc(pose.x, pose.y, radius, 0, 2 * Math.PI, false);
            context.fillStyle = "#ea5e07";
            context.fill();
            context.lineWidth = 1;
            context.strokeStyle = "#262a30";
            context.stroke();
            context.font = "bold 10pt arial";
            context.strokeStyle = "white";
            context.fillStyle = "white";
            context.textAlign = "center";
            context.fillText(i + 1, pose.x, pose.y + 3.5);

            // binding event
            VIZ.bind("click", {
                box: {
                    x: pose.x - radius,
                    y: pose.y - radius,
                    w: radius * 2,
                    h: radius * 2
                },
                handle: function (e) {
                    alertify.message("User clicked on goal " + (i + 1));
                }
            });
        }
    },
    drawPath: function (msg) {
        if (msg.poses.length == 0) return;
        var canvas = VIZ.buffer;
        var ctx = canvas.getContext("2d");
        var i = 0;
        ctx.beginPath();
        while (i < msg.poses.length - 1) {
            var el = msg.poses[i];
            var pose = VIZ.robotPose2Canvas(el.pose.position);
            ctx.moveTo(pose.x, pose.y);
            el = msg.poses[i + 1];
            pose = VIZ.robotPose2Canvas(el.pose.position);
            ctx.lineTo(pose.x, pose.y);
            i++;
        }
        ctx.strokeStyle = "#F48042";
        ctx.stroke();
    },
    drawRobot: function (msg) {
        var canvas = VIZ.buffer;
        var context = canvas.getContext("2d");
        var pose = VIZ.robotPose2Canvas(msg.position);

        context.beginPath();
        var radius = VIZ.robotradius / VIZ.resolution / 2;
        //console.log(radius);
        context.arc(pose.x, pose.y, radius, 0, 2 * Math.PI, false);
        context.fillStyle = "green";
        context.fill();
        context.lineWidth = 1;
        context.strokeStyle = "#003300";
        context.stroke();
    },
    drawMap: function (msg) {
        var canvas = VIZ.buffer;
        canvas.width = msg.info.width;
        canvas.height = msg.info.height;
        VIZ.resolution = msg.info.resolution;
        VIZ.origin.x = -msg.info.origin.position.x / msg.info.resolution;
        VIZ.origin.y = canvas.height + msg.info.origin.position.y / msg.info.resolution;
        var ctx = canvas.getContext("2d");
        // first, create a new ImageData to contain our pixels
        var imgData = ctx.createImageData(msg.info.width, msg.info.height); // width x height
        var data = imgData.data;
        //console.log(imgData);
        // copy img byte-per-byte into our ImageData
        for (var i = 0; i < msg.info.width; i++) {
            for (var j = msg.info.height - 1; j >= 0; j--) {
                var cell = j * msg.info.width + i;
                var index = (msg.info.height - j) * msg.info.width + i;
                switch (msg.data[cell]) {
                    case 0:
                        data[index * 4] = 255;
                        data[index * 4 + 1] = 255;
                        data[index * 4 + 2] = 255;
                        break;
                    case 100:
                        data[index * 4] = 0;
                        data[index * 4 + 1] = 0;
                        data[index * 4 + 2] = 0;
                        break;
                    default:
                        data[index * 4] = 210;
                        data[index * 4 + 1] = 213;
                        data[index * 4 + 2] = 219;
                }
                data[cell * 4 + 3] = 255;
            }
        }

        // now we can draw our imagedata onto the canvas
        ctx.putImageData(imgData, 0, 0);
        VIZ.update();
    },
    update: function () {
        if (!VIZ.buffer || !VIZ.changed) return window.requestAnimationFrame(VIZ.update);
        //console.log("update");
        var canvas = VIZ.canvas;
        var width = VIZ.buffer.width * VIZ.scale;
        var height = VIZ.buffer.height * VIZ.scale;
        canvas.width = width;
        canvas.height = height;
        var ctx = canvas.getContext("2d");
        ctx.save();
        //ctx.translate(-((newWidth-width)/2), -((newHeight-height)/2));
        ctx.scale(VIZ.scale, VIZ.scale);
        ctx.clearRect(0, 0, width, height);
        ctx.drawImage(VIZ.buffer, 0, 0);
        ctx.restore();
        VIZ.changed = false;
        window.requestAnimationFrame(VIZ.update);
    },
    zoom: function (delta) {
        VIZ.scale += 0.2 * delta;
    }
};


var isFullscreen = false;

var enterFullscreen = function () {
    var el = $("body")[0];
    if( isFullscreen )
    {
        isFullscreen = false;
        if(document.exitFullscreen)
            return document.exitFullscreen();
        if(document.mozCancelFullScreen)
            return document.mozCancelFullScreen() ;
        if(document.webkitExitFullscreen)   
            return document.webkitExitFullscreen();
        if(document.cancelFullScreen)
            return document.cancelFullScreen();
    }
    isFullscreen = true;
    if (el.requestFullscreen)
        return el.requestFullscreen();
    else if (el.mozRequestFullScreen)
        return el.mozRequestFullScreen();
    else if (el.webkitRequestFullscreen)
        return el.webkitRequestFullscreen();
    else if (el.msRequestFullscreen)
        return el.msRequestFullscreen();
};
window.mobilecheck = function () {
    if (navigator.userAgent.match(/Android/i)
        || navigator.userAgent.match(/webOS/i)
        || navigator.userAgent.match(/iPhone/i)
        || navigator.userAgent.match(/iPad/i)
        || navigator.userAgent.match(/iPod/i)
        || navigator.userAgent.match(/BlackBerry/i)
        || navigator.userAgent.match(/Windows Phone/i)
    ) {
        return true;
    }
    else {
        return false;
    }
};

var initAndroidWebSocket = function()
{

        if(! window.WebSocketFactory) return;
        // window object
        var global = window;
        
        // WebSocket Object. All listener methods are cleaned up!
        var WebSocket = global.WebSocket = function(url) {
            // get a new websocket object from factory (check com.strumsoft.websocket.WebSocketFactory.java)
            this.socket = WebSocketFactory.getInstance(url);
            // store in registry
            if(this.socket) {
                WebSocket.store[this.socket.getId()] = this;
            } else {
                throw new Error("Websocket instantiation failed! Address might be wrong.");
            }
        };
        
        // storage to hold websocket object for later invokation of event methods
        WebSocket.store = {};
        
        // static event methods to call event methods on target websocket objects
        WebSocket.onmessage = function (evt) {
            WebSocket.store[evt._target]["onmessage"].call(global, evt);
        }	
        
        WebSocket.onopen = function (evt) {
            WebSocket.store[evt._target]["onopen"].call(global, evt);
        }
        
        WebSocket.onclose = function (evt) {
            WebSocket.store[evt._target]["onclose"].call(global, evt);
        }
        
        WebSocket.onerror = function (evt) {
            WebSocket.store[evt._target]["onerror"].call(global, evt);
        }
    
        // instance event methods
        WebSocket.prototype.send = function(data) {
            this.socket.send(data);
        }
    
        WebSocket.prototype.close = function() {
            this.socket.close();
        }
        
        WebSocket.prototype.getReadyState = function() {
            this.socket.getReadyState();
        }
        ///////////// Must be overloaded
        WebSocket.prototype.onopen = function(){
            throw new Error("onopen not implemented.");
        };
        
        // alerts message pushed from server
        WebSocket.prototype.onmessage = function(msg){
            throw new Error("onmessage not implemented.");
        };
        
        // alerts message pushed from server
        WebSocket.prototype.onerror = function(msg){
            throw new Error("onerror not implemented.");
        };
        
        // alert close event
        WebSocket.prototype.onclose = function(){
            throw new Error("onclose not implemented.");
        };
};

var tesws = function()
{
    var websocket = new WebSocket("wss://echo.websocket.org"); 
	websocket.onopen = function(evt) {alertify.success("Ws connected");websocket.send("test data"); }; 
	websocket.onclose = function(evt) { alertify.warning("Ws close"); }; 
	websocket.onmessage = function(evt) { alertify.message("Ws msg" + evt.data); websocket.close(); }; 
	websocket.onerror = function(evt) { alertify.error("ERROR: Ws problem"); }; 
    alertify.message("End Init the websocket");
    
};

var boot = function()
{
    initAndroidWebSocket();
    
    $("#fullscreenbt").click(function (e) {
        enterFullscreen();
    });
    $("#zoomin").click(function (e) {
        VIZ.zoom(1);
    });
    $("#zoomout").click(function (e) {
        VIZ.zoom(-1);
    });
    // mouse event
    var hamster = Hamster($("#canvas")[0]);
    hamster.wheel(function (event, delta, deltaX, deltaY) {
        VIZ.zoom(delta);
    });

    VIZ.buffer = $("<canvas>")[0];
    VIZ.canvas = $("#canvas")[0];
    alertify.defaults.glossary.title = "VIZ";
    VIZ.main();
    //tesws()

};
$(window).on("load", boot);