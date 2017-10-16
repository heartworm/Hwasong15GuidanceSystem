Vue.component('config', {
    template: 
`   
<ul  class="config__item pure-u-1">
    <div v-if="loaded">
        <li v-if="error">Error check console.</li>
        <li class="config-body" v-else>
            <form v-on:submit.prevent="send" class="pure-form" v-if="isVal">
                <label v-bind:for="path">{{name}}</label>
                <input v-if="type == 'number'" v-bind:name="path" type="number" step="any" v-model.number="val">
                <input v-if="type==='string'" v-bind:name="path" type="text" v-model="val">
                <input v-bind:disabled="sending" v-bind:value="sending ? 'Sending' : 'Send'" class="pure-button" type="submit">
            </form>
            <div v-else>
                <a href="#" v-on:click.prevent="loaded=false"><strong>{{name}}</strong></a>
                <config v-for="pathEnd in val" v-bind:name="pathEnd" v-bind:path="path + pathEnd + '/'"></config>
            </div>
        </li>
    </div>
    <div v-else>
        <li> <a href="#" v-on:click.prevent="load"><strong>{{name}}</strong></a>
    </div>
</ul>
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
            xhr.open("GET", "/api/config" + this.path);
            xhr.responseType = "json";
            xhr.onload = (e) => {
                console.log(this);
                if (!this.loaded) {
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
                }
            };
            xhr.send();
        },
        send: function() {
            var xhr = new XMLHttpRequest();
            xhr.open("POST", "/api/config" + this.path);
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