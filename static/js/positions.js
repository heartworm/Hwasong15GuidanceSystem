Vue.component('positions', {
    template: 
`
    <div>
        <canvas class="pure-u-1" ref="positionCanvas"></canvas>
    </div>
`,
    props: ['positions'],
    data: function() {
        return {
            chart: null,
            ball: {
                label: 'Ball',
                data: [],
                pointBackgroundColor: 'rgb(255,165,0)'
            },
            walls: {
                label: 'Walls',
                data: [],
                pointBackgroundColor: 'rgb(0,0,255)'
            }
        };
    },
    methods: {
        drawPositions: function() {  
            this.ball.data = [];
            if (this.positions.ballPos != null) {
                var ballXY = this.toXY(this.positions.ballPos);
                if (ballXY != null) this.ball.data.push(ballXY);
            }
            
            this.walls.data = [];
            for (point of this.positions.wallPoses) {
                var xy = this.toXY(point);
                if (xy != null) {
                    this.walls.data.push(xy);
                }
            }
            this.chart.update();
        },
        toDataset: function(name, data) {
            var chartData = [];
            if (!Array.isArray(data)) {
                data = [data];
            }
            for (point of data) {
                var xy = this.toXY(point);
                if (xy != null) {
                    chartData.push(xy);
                }
            }
            return {
                label: name,
                data: chartData
            }
        },
        toXY: function(point) {
            if (point.cartesian[0] == null) return null;
            return {
                x: point.cartesian[0],
                y: point.cartesian[1]
            }
        }
    },
    watch: {
        positions: function() {
            this.drawPositions();
        }
    },
    mounted: function() {
        var canvas = this.$refs.positionCanvas;
        var ctx = canvas.getContext('2d');
        Chart.defaults.global.responsive = true;
        this.chart = new Chart(ctx, {
            type: 'scatter',
            data: {
                datasets: [
                    this.ball,
                    this.walls,
                ]
            },
            options: {
                responsive: true,
                animation: false,
                scales: {
                    yAxes: [{
                        ticks: {
                            max: 2,
                            min: -2
                        }
                    }],
                    xAxes: [{
                        ticks: {
                            max: 2,
                            min: -2
                        }   
                    }]
                }
            }
        }); 
    }
})