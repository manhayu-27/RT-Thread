package com.example.esp32monitor

import android.Manifest
import android.annotation.SuppressLint
import android.app.Activity
import android.app.AlertDialog
import android.content.Intent
import android.content.pm.PackageManager
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager
import android.net.Uri
import android.os.Bundle
import android.provider.Settings
import android.text.InputType
import android.webkit.JavascriptInterface
import android.webkit.WebChromeClient
import android.webkit.WebView
import android.webkit.WebViewClient
import android.widget.Toast
import android.widget.EditText
import androidx.activity.ComponentActivity
import org.json.JSONArray
import org.json.JSONObject
import java.io.File
import java.io.FileWriter
import java.time.ZonedDateTime
import java.time.format.DateTimeFormatter
import java.util.Locale
import java.util.UUID
import java.net.HttpURLConnection
import java.net.URL

class MainActivity : ComponentActivity(), LocationListener {
    private lateinit var webView: WebView
    private val taskDir by lazy { File(filesDir, "data").apply { mkdirs() } }
    private var exportFile: File? = null
    private val preferences by lazy { getSharedPreferences("bioscope", MODE_PRIVATE) }

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        webView = WebView(this).apply {
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true
            settings.allowFileAccess = true
            addJavascriptInterface(Bridge(), "AndroidHost")
            webViewClient = WebViewClient()
            webChromeClient = WebChromeClient()
            loadUrl("file:///android_asset/index.html")
        }
        setContentView(webView)
    }

    override fun onDestroy() {
        (getSystemService(LOCATION_SERVICE) as LocationManager).removeUpdates(this)
        webView.destroy()
        super.onDestroy()
    }

    private fun now(): String = DateTimeFormatter.ISO_OFFSET_DATE_TIME.format(ZonedDateTime.now())

    private fun taskFile(id: String, extension: String): File {
        require(id.matches(Regex("[A-Za-z0-9_-]+"))) { "invalid task id" }
        return File(taskDir, "$id.$extension")
    }

    private fun publicMeta(meta: JSONObject): JSONObject = JSONObject().apply {
        listOf("id", "name", "note", "status", "createdAt", "startedAt", "endedAt", "sampleRate", "sampleCount", "alarmCount", "durationSeconds")
            .forEach { key -> put(key, meta.opt(key)) }
    }

    private fun listTasks(): JSONObject {
        val tasks = taskDir.listFiles { file -> file.extension == "json" }?.mapNotNull {
            runCatching { publicMeta(JSONObject(it.readText())) }.getOrNull()
        }?.sortedByDescending { it.optString("createdAt") } ?: emptyList()
        return JSONObject().put("tasks", JSONArray().apply { tasks.forEach(::put) })
    }

    private fun createTask(body: JSONObject): JSONObject {
        val name = body.optString("name").trim()
        val note = body.optString("note").trim()
        require(name.isNotEmpty() && name.length <= 80 && note.length <= 500) { "任务名称或备注无效" }
        val id = "${System.currentTimeMillis()}_${UUID.randomUUID().toString().take(6)}"
        val meta = JSONObject().apply {
            put("id", id); put("name", name); put("note", note); put("status", "recording")
            put("createdAt", now()); put("startedAt", now()); put("startedAtMs", System.currentTimeMillis())
            put("endedAt", JSONObject.NULL); put("sampleRate", body.optInt("sampleRate", 500))
            put("sampleCount", 0); put("alarmCount", 0); put("durationSeconds", 0)
        }
        taskFile(id, "csv").writeText("sample_index,timestamp_us,ecg_mv,emg1_mv,emg2_mv,emg3_mv,gyro_x_dps,gyro_y_dps,gyro_z_dps,fall,roll_deg,pitch_deg,yaw_deg,temperature_c,alarm_flags\n")
        taskFile(id, "json").writeText(meta.toString())
        return publicMeta(meta)
    }

    private fun appendSamples(id: String, body: JSONObject): JSONObject {
        val rows = body.getJSONArray("samples")
        require(rows.length() <= 5000) { "samples too large" }
        val meta = JSONObject(taskFile(id, "json").readText())
        require(meta.optString("status") == "recording") { "任务未在采集中" }
        var count = meta.optInt("sampleCount")
        var alarms = meta.optInt("alarmCount")
        FileWriter(taskFile(id, "csv"), true).use { writer ->
            for (rowIndex in 0 until rows.length()) {
                val row = rows.getJSONArray(rowIndex)
                require(row.length() >= 14) { "sample format invalid" }
                val flags = row.optInt(13)
                writer.append(count.toString())
                for (column in 0 until 13) writer.append(',').append(row.opt(column).toString())
                writer.append(',').append(flags.toString()).append('\n')
                count++
                if (flags != 0) alarms++
            }
        }
        meta.put("sampleCount", count).put("alarmCount", alarms)
        taskFile(id, "json").writeText(meta.toString())
        return publicMeta(meta)
    }

    private fun stopTask(id: String): JSONObject {
        val meta = JSONObject(taskFile(id, "json").readText())
        meta.put("status", "completed").put("endedAt", now())
            .put("durationSeconds", ((System.currentTimeMillis() - meta.optLong("startedAtMs")) / 1000).coerceAtLeast(0))
        taskFile(id, "json").writeText(meta.toString())
        return publicMeta(meta)
    }

    private fun taskData(id: String): JSONObject {
        val meta = JSONObject(taskFile(id, "json").readText())
        val total = meta.optInt("sampleCount")
        val stride = maxOf(1, (total + 11999) / 12000)
        val samples = JSONArray()
        taskFile(id, "csv").useLines { lines ->
            lines.drop(1).forEachIndexed { index, line ->
                if (index % stride != 0 && index != total - 1) return@forEachIndexed
                val values = line.split(',')
                if (values.size != 15) return@forEachIndexed
                val row = JSONArray()
                row.put(values[0].toInt()).put(values[1].toLong())
                for (column in 2..13) row.put(values[column].toDouble())
                row.put(values[14].toInt())
                samples.put(row)
            }
        }
        return JSONObject().put("task", publicMeta(meta)).put("stride", stride).put("samples", samples)
    }

    private fun deleteTask(id: String): JSONObject {
        val meta = JSONObject(taskFile(id, "json").readText())
        require(meta.optString("status") != "recording") { "正在采集的任务不能删除" }
        taskFile(id, "json").delete(); taskFile(id, "csv").delete()
        return JSONObject().put("ok", true)
    }

    private fun taskSummary(id: String): String {
        val meta = JSONObject(taskFile(id, "json").readText())
        val sums = DoubleArray(4)
        val squares = DoubleArray(4)
        var count = 0
        var alarms = 0
        taskFile(id, "csv").useLines { lines -> lines.drop(1).forEach { line ->
            val values = line.split(',')
            if (values.size != 15) return@forEach
            for (index in 0..3) {
                val value = values[index + 2].toDouble()
                sums[index] += value; squares[index] += value * value
            }
            if (values[14].toInt() != 0) alarms++
            count++
        } }
        require(count > 0) { "该任务还没有可分析的信号数据" }
        fun stats(index: Int) = String.format(Locale.US, "mean=%.4f mV, rms=%.4f mV", sums[index] / count, kotlin.math.sqrt(squares[index] / count))
        return "任务：${meta.optString("name")}；样本数：$count；报警样本：$alarms；ECG ${stats(0)}；EMG1 ${stats(1)}；EMG2 ${stats(2)}；EMG3 ${stats(3)}。仅用于工程调试或教学研究，不用于医学诊断。"
    }

    private fun callArk(messages: JSONArray): String {
        val key = preferences.getString("arkKey", "")?.trim().orEmpty()
        require(key.isNotEmpty()) { "请先点击 AI 状态配置 ARK_API_KEY" }
        val request = JSONObject().put("model", "doubao-seed-2-0-lite-260215").put("messages", messages).put("max_tokens", 1600)
        val connection = URL("https://ark.cn-beijing.volces.com/api/v3/chat/completions").openConnection() as HttpURLConnection
        return try {
            connection.requestMethod = "POST"; connection.connectTimeout = 15000; connection.readTimeout = 60000
            connection.setRequestProperty("Authorization", "Bearer $key")
            connection.setRequestProperty("Content-Type", "application/json")
            connection.doOutput = true
            connection.outputStream.bufferedWriter().use { it.write(request.toString()) }
            val body = (if (connection.responseCode in 200..299) connection.inputStream else connection.errorStream).bufferedReader().use { it.readText() }
            val response = JSONObject(body)
            if (connection.responseCode !in 200..299) throw IllegalStateException(response.optJSONObject("error")?.optString("message") ?: "AI 请求失败")
            response.getJSONArray("choices").getJSONObject(0).getJSONObject("message").getString("content")
        } finally { connection.disconnect() }
    }

    private inner class Bridge {
        @JavascriptInterface fun api(path: String, method: String, body: String): String = try {
            val result = when {
                path == "/api/health" -> JSONObject().put("ok", true)
                path == "/api/ai/status" -> JSONObject().put("configured", preferences.contains("arkKey")).put("model", "doubao-seed-2-0-lite-260215")
                path == "/api/tasks" && method == "GET" -> listTasks()
                path == "/api/tasks" && method == "POST" -> createTask(JSONObject(body))
                Regex("/api/tasks/[^/]+/samples").matches(path) && method == "POST" -> appendSamples(path.split('/')[3], JSONObject(body))
                Regex("/api/tasks/[^/]+/stop").matches(path) && method == "POST" -> stopTask(path.split('/')[3])
                Regex("/api/tasks/[^/]+/data").matches(path) && method == "GET" -> taskData(path.split('/')[3])
                Regex("/api/tasks/[^/]+").matches(path) && method == "DELETE" -> deleteTask(path.split('/')[3])
                path == "/api/ai/report" && method == "POST" -> JSONObject().put("report", callArk(JSONArray().put(JSONObject().put("role", "user").put("content", "根据以下采集统计生成简洁观察报告，必须说明不用于医学诊断：" + taskSummary(JSONObject(body).getString("taskId"))))))
                path == "/api/ai/chat" && method == "POST" -> {
                    val payload = JSONObject(body)
                    val messages = JSONArray()
                    payload.optString("taskId").takeIf { it.isNotBlank() }?.let { messages.put(JSONObject().put("role", "system").put("content", taskSummary(it))) }
                    payload.optJSONArray("history")?.let { history -> for (index in 0 until minOf(history.length(), 10)) messages.put(history.getJSONObject(index)) }
                    messages.put(JSONObject().put("role", "user").put("content", payload.getString("message")))
                    JSONObject().put("answer", callArk(messages))
                }
                else -> JSONObject().put("error", "not found")
            }
            result.toString()
        } catch (error: Exception) {
            JSONObject().put("error", error.message ?: "操作失败").toString()
        }

        @JavascriptInterface fun startLocation() = runOnUiThread { requestPhoneLocation() }

        @JavascriptInterface fun configureAi() = runOnUiThread {
            val input = EditText(this@MainActivity).apply {
                inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_PASSWORD
                setText(preferences.getString("arkKey", ""))
                hint = "ARK_API_KEY"
            }
            AlertDialog.Builder(this@MainActivity).setTitle("配置豆包 ARK API 密钥").setView(input)
                .setPositiveButton("保存") { _, _ -> preferences.edit().putString("arkKey", input.text.toString().trim()).apply(); webView.evaluateJavascript("checkAiStatus()", null) }
                .setNegativeButton("取消", null).show()
        }

        @JavascriptInterface fun exportCsv(id: String) = runOnUiThread {
            exportFile = taskFile(id, "csv")
            startActivityForResult(Intent(Intent.ACTION_CREATE_DOCUMENT).apply {
                type = "text/csv"; putExtra(Intent.EXTRA_TITLE, "$id.csv")
                addCategory(Intent.CATEGORY_OPENABLE)
            }, 7)
        }
    }

    private fun requestPhoneLocation() {
        if (checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(arrayOf(Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION), 6)
            return
        }
        val manager = getSystemService(LOCATION_SERVICE) as LocationManager
        if (!manager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
            Toast.makeText(this, "请打开手机定位服务", Toast.LENGTH_LONG).show()
            startActivity(Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS))
            return
        }
        manager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000L, 0f, this)
        manager.getLastKnownLocation(LocationManager.GPS_PROVIDER)?.let { location -> onLocationChanged(location) }
    }

    override fun onLocationChanged(location: Location) {
        webView.post {
            webView.evaluateJavascript(
                String.format(Locale.US, "window.onPhoneLocation(%.7f,%.7f,%.1f)", location.latitude, location.longitude, location.accuracy),
                null,
            )
        }
    }

    override fun onRequestPermissionsResult(requestCode: Int, permissions: Array<out String>, results: IntArray) {
        super.onRequestPermissionsResult(requestCode, permissions, results)
        if (requestCode == 6 && results.any { it == PackageManager.PERMISSION_GRANTED }) requestPhoneLocation()
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)
        if (requestCode == 7 && resultCode == Activity.RESULT_OK) {
            data?.data?.let { uri -> exportFile?.let { copyCsv(it, uri) } }
        }
        exportFile = null
    }

    private fun copyCsv(source: File, target: Uri) {
        contentResolver.openOutputStream(target)?.use { output -> source.inputStream().use { it.copyTo(output) } }
    }

}
