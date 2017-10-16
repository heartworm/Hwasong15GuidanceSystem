Vue.component('videos', {
    template: 
`
    <div>
        <div class="pure-u-1 pure-u-lg-2-5">
            <div class="pure-g">
                <button class="pure-button pure-u-1" v-for="video in videos" v-on:click="playVideo(video);" v-bind:class="{'pure-button-active':currentVideo==video}">
                    {{video}}
                </button>
                <button class="pure-button pure-button-primary pure-u-1" v-on:click="currentVideo=null">
                        Stop
                </button>
            </div>
        </div>
        <div class="pure-u-1 pure-u-lg-3-5">
            <img class="pure-img videos__img" v-bind:src="'/api/video/' + currentVideo" v-if="currentVideo != null" />   
        </div>
    </div>
`,
    props: ['videos'],
    data: function() {
        return {
            currentVideo: null
        }
    },
    methods: {
        playVideo: function(video) {
            this.currentVideo = video;
        }
    }
});
