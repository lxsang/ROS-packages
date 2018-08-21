
VIZ = {
    buffer: {},
    scale: 1.0,
    drawMap: function (msg) {
        VIZ.buffer = $("<canvas>")[0];
        var canvas = VIZ.buffer;
        canvas.width = msg.info.width;
        canvas.height = msg.info.height;
        var ctx = canvas.getContext('2d');
        // first, create a new ImageData to contain our pixels
        var imgData = ctx.createImageData(msg.info.width, msg.info.height); // width x height
        var data = imgData.data;
        console.log(imgData);
        // copy img byte-per-byte into our ImageData
        for (var i = 0; i < msg.info.width; i++) {
            for (var j = msg.info.height - 1; j >= 0; j--) {
                var cell = j*msg.info.width + i;
                var index = (msg.info.height - j)*msg.info.width + i;
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
                        data[index * 4] = 173;
                        data[index * 4 + 1] = 175;
                        data[index * 4 + 2] = 178;
                }
                data[cell * 4 + 3] = 255;
            }
        }

        // now we can draw our imagedata onto the canvas
        ctx.putImageData(imgData, 0, 0);
        VIZ.update();
    },
    update:function()
    {
        console.log("update");
        var canvas = $("#canvas")[0];
        var width = VIZ.buffer.width*VIZ.scale;
        var height = VIZ.buffer.height*VIZ.scale;
        canvas.width = width;
        canvas.height = height;
        var ctx = canvas.getContext('2d');
        ctx.save();
        //ctx.translate(-((newWidth-width)/2), -((newHeight-height)/2));
        ctx.scale(VIZ.scale, VIZ.scale);
        ctx.clearRect(0, 0, width, height);
        ctx.drawImage(VIZ.buffer, 0, 0);
        ctx.restore();
    }
}
var canvas_scale = 1;
$(window).on('load', function () {

    var hamster = Hamster($("#canvas")[0]);

    hamster.wheel(function(event, delta, deltaX, deltaY){
        VIZ.scale += 0.2*delta;
        VIZ.update();
    });
    var server = 'ws://localhost:9090';
    //subscriber to topic map
    var ros = new ROSLIB.Ros({
        url: server
    });

    ros.on('connection', function () {
        console.log('Connected to websocket server.');
    });

    ros.on('error', function (error) {
        console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', function () {
        console.log('Connection to websocket server closed.');
    });

    var listener = new ROSLIB.Topic({
        ros: ros,
        name: '/map',
        messageType: 'nav_msgs/OccupancyGrid'
    });

    listener.subscribe(function (message) {
        VIZ.drawMap(message);
        //listener.unsubscribe();
        //ros.close()
    });

});