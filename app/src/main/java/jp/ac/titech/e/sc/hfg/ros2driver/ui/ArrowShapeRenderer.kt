import android.graphics.Canvas
import android.graphics.Paint
import android.graphics.Path
import com.github.mikephil.charting.renderer.scatter.IShapeRenderer
import com.github.mikephil.charting.utils.ViewPortHandler
import com.github.mikephil.charting.interfaces.datasets.IScatterDataSet
import com.github.mikephil.charting.utils.Transformer
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

class ArrowShapeRenderer(private val transformer: Transformer) : IShapeRenderer {

    override fun renderShape(
        c: Canvas,
        dataSet: IScatterDataSet,
        viewPortHandler: ViewPortHandler,
        posX: Float,
        posY: Float,
        renderPaint: Paint
    ) {
        val entry = dataSet.getEntryForXValue(posX, posY)

        if (entry.data is FloatArray && (entry.data as FloatArray).size == 2) {
            val endData = entry.data as FloatArray
            val pts = floatArrayOf(endData[0], endData[1])
            transformer.pointValuesToPixel(pts)
            val endX = pts[0]
            val endY = pts[1]

            c.drawLine(posX, posY, endX, endY, renderPaint)

            val angle = atan2((endY - posY), (endX - posX))
            val arrowHead = Path().apply {
                moveTo(endX, endY)
                lineTo(
                    endX - 20 * cos(angle - Math.PI / 6).toFloat(),
                    endY - 20 * sin(angle - Math.PI / 6).toFloat()
                )
                lineTo(
                    endX - 20 * cos(angle + Math.PI / 6).toFloat(),
                    endY - 20 * sin(angle + Math.PI / 6).toFloat()
                )
                close()
            }

            c.drawPath(arrowHead, renderPaint)
        }
    }
}
