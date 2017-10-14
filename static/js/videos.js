Vue.component('videos', {
    template: 
`
ASSDFSADFAS
<div v-if="loaded">
    <div v-if="error">
        <p>Error loading videos check console.</p>
    </div>
    <div v-else class="pure-g">
        <div class="pure-u-1 pure-u-lg-1-5">
            <div class="pure-g">
                <button class="pure-button pure-u-1" v-for="video in videos" v-on:click="playVideo(video);" v-bind:class="{'pure-button-active':currentVideo==video}">
                    {{video}}
                </button>
                <button class="pure-button pure-button-primary pure-u-1" v-on:click="currentVideo=null">
                        Stop
                </button>
                <button class="pure-button pure-button-primary pure-u-1" v-on:click="loadVideos">
                    Reload
                </button>

            </div>
        </div>
        <div class="pure-u-1 pure-u-lg-4-5">
            <img class="pure-u-4-5 pure-img videos__img" v-bind:src="'/api/video/' + currentVideo" v-if="currentVideo != null" />   
        </div>
    </div>
</div>
<div v-else>
    <p v-if="loading">Loading videos...</p>
    <a v-else href="#" v-on:click="loadVideos">Load Videos</a>
</div>
`,
    data: function() {
        return {
            currentVideo: null,
            videos: null,
            loaded: false,
            loading: false,
            error: false,
            currentVideo: null,
        }
    },
    methods: {
        loadVideos: function() {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/api/video");
            xhr.responseType = "json";
            xhr.onload = (e)  => {
                if (this.loading) {
                    this.loading = false;
                    var status = e.target.status;
                    if (status == 200) {
                        this.error = false;
                        this.videos = e.target.response;
                    } else {
                        this.error = true;
                        console.log(e);
                    }
                    this.loaded = true;
                }
            };
            
            this.loaded = false;
            this.loading = true;
            xhr.send();
        },
        playVideo: function(video) {
//            window.location = '/api/video/' + video;
            this.currentVideo = video;
        }
    },
    created: function() {
        this.loadVideos();
    }
});
