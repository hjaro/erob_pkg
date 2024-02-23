var ros = new ROSLIB.Ros();

const app = Vue.createApp({

    el: "#app",
    data() {
        return {
            // ros connection
            ros: null,
            rosbridge_address: 'ws://127.0.0.1:9090/',
            connected: false,
            error: false,
            // subscriber data
            position: [0, 0, 0, 0, 0, 0],
            j1: 0,
            j2: 0,
            j3: 0,
            j4: 0,
            j5: 0,
            j6: 0,
            history: [],
            
            // history: [{
            //     joint1: this.j1, joint1: this.j2,
            //      joint3: this.j3, joint1: this.j4, 
            //      joint5: this.j5, joint1: this.j6
            // }],
            // page content
            menu_title: 'Connection',
            main_title: 'Main title, from Vue!!',
        }
    },
    methods: {
        connect: function () {
            // define ROSBridge connection object
            this.ros = new ROSLIB.Ros({
                url: this.rosbridge_address
            })

            // define callbacks
            this.ros.on('connection', () => {
                this.connected = true
                console.log('Connection to ROSBridge established!')

                this.showPosition()

            })
            this.ros.on('error', (error) => {
                this.error = true
                console.log('Something went wrong when trying to connect')
                console.log(error)
            })
            this.ros.on('close', () => {
                this.connected = false
                console.log('Connection to ROSBridge was closed!')
            })
        },
        disconnect: function () {
            this.ros.close()
        },
        sendCommand: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: 'arm/command',
                messageType: 'sensor_msgs/msg'
            })
            let message = new ROSLIB.Message({
                linear: { x: 1, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: 0.5, },
            })
            topic.publish(message)
        },
        turnRight: function () {
            let topic = new ROSLIB.Topic({
                ros: this.ros,
                name: '/cmd_vel',
                messageType: 'geometry_msgs/Twist'
            })
            let message = new ROSLIB.Message({
                linear: { x: 1, y: 0, z: 0, },
                angular: { x: 0, y: 0, z: -0.5, },
            })
            topic.publish(message)
        },
        Stop: function () {
            var cmdVel = new ROSLIB.Topic({
                ros: ros,
                name: "/arm/stop",
                messageType: "std_msgs/msg/String",
            });
            var msg = new ROSLIB.Message({
                data: "stop"
            });

            cmdVel.publish(msg);
        },
        Disengage: function () {
            var cmdVel = new ROSLIB.Topic({
                ros: ros,
                name: "/arm/disengage",
                messageType: "std_msgs/msg/String",
            });
            var msg = new ROSLIB.Message({
                data: "disengage"
            });

            cmdVel.publish(msg);
        },
        addItem: function () {

            let item = {
                joint1: this.j1,
                joint2: this.j2,
                joint3: this.j3,
                joint4: this.j4,
                joint5: this.j5,
                joint6: this.j6
            }
            this.history.push(item)

            this.$refs.table.scrollIntoView({ block: "end" });
        },
        showPosition: function () {
            var listener = new ROSLIB.Topic({
                ros: ros,
                name: "/arm/state",
                messageType: "sensor_msgs/msg/JointState",
            });

            listener.subscribe(function (message) {

                var poses = message.position;
                // console.log(poses);
                let txt = "";
                let i = 1;
                for (let x in poses) {
                    p = ((poses[x] * 180) / Math.PI) % 360;
                    if (p > 180) {
                        p = p - 360;
                    }
                    position[x] = p;
                }
                listener.unsubscribe();
            });
        },
        send:function() {
            var cmdVel = new ROSLIB.Topic({
                ros: ros,
                name: "/arm/command",
                messageType: "sensor_msgs/msg/JointState",
            });

           

            rj1 = (this.j1 * Math.PI) / 180;
            rj2 = (this.j2 * Math.PI) / 180;
            rj3 = (this.j3 * Math.PI) / 180;
            rj4 = (this.j4 * Math.PI) / 180;
            rj5 = (this.j5 * Math.PI) / 180;
            rj6 = (this.j6 * Math.PI) / 180;

            var JointState = new ROSLIB.Message({
                name: ["j1", "j2", "j3", "j4", "j5", "j6"],
                position: [this.j1, this.j2, this.j3, this.j4, this.j5, this.j6],
                velocity: [10, 10, 10, 10, 10, 10],
                effort: [],
            });
            cmdVel.publish(JointState);

            this.addItem()

        }

    },

    mounted() {
        // page is ready
        console.log('page is ready!')
    },
})

app.mount('#app')