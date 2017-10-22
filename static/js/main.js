var app = new Vue({
    el: '#app',
    data: {
        error: false,
        status: null
    },
    methods: {
        getStatus: function() {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/api/status");
            xhr.responseType = "json";
            xhr.onload = (e) => {
                console.log(e.target.response);
                console.log(this.status);
                if (e.target.status == 200) {
                    this.error = false;
                    if (this.status == null) {
                        this.status = e.target.response;
                    } else {
                        Object.assign(this.status, e.target.response);
                    }
                } else {
                    this.onError(e);
                }
                window.setTimeout(this.getStatus, 250);
//                this.getStatus();
            };
            xhr.onerror = (e) => {
                this.onError(e);
            }
            xhr.send();
        },
        blue: function() {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/api/blue");
            xhr.send();
        },
        yellow: function() {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/api/yellow");
            xhr.send();
        },
        stop: function() {
           var xhr = new XMLHttpRequest();
            xhr.open("GET", "/api/stop");
            xhr.send();
        },
        onError: function(event) {
            console.log(event);
            this.started = false;
            this.error = true;
        }
    },
    created: function() {
        this.getStatus();
    }
});
