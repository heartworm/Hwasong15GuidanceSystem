var app = new Vue({
    el: '#app',
    data: {
        started: false,
        error: false,
    },
    methods: {
        start: function() {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/api/start");
            xhr.onload = (e) => {
                if (e.target.status == 200) {
                    this.started = true;
                } else {
                    console.log(e);
                    this.error = true;
                    this.started = false;
                }
            }
            xhr.send();
        },
        stop: function() {
           var xhr = new XMLHttpRequest();
            xhr.open("GET", "/api/stop");
            xhr.onload = (e) => {
                if (e.target.status == 200) {
                    this.started = false;
                } else {
                    console.log(e);
                    this.started = false;
                    this.error = true;
                }
            }
            xhr.send();
        }
    }
});
