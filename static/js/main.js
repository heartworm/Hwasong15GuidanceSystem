Vue.component('config', {
    template: 
`   
<div class="config__item pure-u-1">
    <div v-if="loaded" class="config__item pure-u-1">
        <p v-if="error">Error check console.</p>
        <div class="config-body" v-else>
            <form v-on:submit.prevent="send" class="pure-form pure-form-stacked" v-if="isVal">
                <span v-if="sending">Sending</span>
                <label v-bind:for="path">{{name}}</label>
                <div v-if="type == 'number'">
                    <input v-bind:name="path" type="number" step="any" v-model.number="val">
                </div>
                <div v-if="type == 'string'">
                    <input v-bind:name="path" type="text" v-model="val">
                </div>
                <input v-bind:disabled="sending" v-bind:value="sending ? 'Sending' : 'Send'" class="pure-button" type="submit">
            </form>
            <div v-else>
                <p>
                    <strong>{{name}}</strong>
                    <button class="pure-button" v-on:click="loaded=false;">Collapse</button>
                </p>
                <config v-for="pathEnd in val" v-bind:name="pathEnd" v-bind:path="path + pathEnd + '/'"></config>
            </div>
        </div>
    </div>
    <div v-else>
        <p>
            <strong>{{name}}</strong>
            <button class="pure-button" v-on:click="load">Expand</button>
        </p>
    </div>
</div>
`,
    props: ['path', 'name'],
    data: function() {
        return {
            loaded: false,
            sending: false,
            isVal: null,
            val: null,
            error: null,
            type: null
        }
    },
    methods: {
        loadVal: function(val) {
            this.val = val;
            this.isVal = !Array.isArray(this.val);
            this.type = typeof val;
        },
        load: function() {
            console.log(this);
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/config" + this.path);
            xhr.responseType = "json";
            xhr.onload = (e) => {
                status = e.target.status;
                if (status == 200) {
                    json = e.target.response;
                    this.error = false;
                    this.loadVal(json);
                    this.loaded = true;
                } else {
                    console.log(true);
                    this.error = true;
                    this.loaded = true;
                }
            };
            xhr.send();
        },
        send: function() {
            var xhr = new XMLHttpRequest();
            xhr.open("POST", "/config" + this.path);
            this.sending = true;
            xhr.onload = (e) => {
                status = e.target.status;
                if (status == 200) {
                    this.sending = false;
                } else {
                    console.log(e);
                    this.error = true;
                }
            };
            xhr.send(JSON.stringify(this.val));
        }
    }
});


var app = new Vue({
    el: '#app',
    data: {
    }
});
